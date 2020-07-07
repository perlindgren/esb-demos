#![no_std]
#![no_main]

// Import the right HAL/PAC crate, depending on the target chip
#[cfg(feature = "51")]
use nrf51_hal as hal;
#[cfg(feature = "52810")]
use nrf52810_hal as hal;
#[cfg(feature = "52832")]
use nrf52832_hal as hal;
#[cfg(feature = "52840")]
use nrf52840_hal as hal;

use {
    core::{
        default::Default,
        fmt::Write,
        panic::PanicInfo,
        sync::atomic::{compiler_fence, AtomicBool, Ordering},
    },
    esb::{
        consts::*, irq::StatePRX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
        EsbBuffer, EsbHeader, EsbIrq, IrqTimer,
    },
    hal::{gpio::Level, pac::TIMER0},
    rtt_target::{rprintln, rtt_init_print},
};

const MAX_PAYLOAD_SIZE: u8 = 64;
const RESP: &'static str = "Hello back";

#[rtic::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        esb_app: EsbApp<U1024, U1024>,
        esb_irq: EsbIrq<U1024, U1024, TIMER0, StatePRX>,
        esb_timer: IrqTimer<TIMER0>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let _clocks = hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();
        rtt_init_print!();
        rprintln!("RTX init");

        // We statically allocate space for buffers.
        static BUFFER: EsbBuffer<U1024, U1024> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };

        // ESB default pipe address mapping
        let addresses = Addresses::default();

        // ESB driver configuration
        // Default is to listen to packes on all pipes
        let config = ConfigBuilder::default()
            .maximum_transmit_attempts(1)
            .max_payload_size(MAX_PAYLOAD_SIZE)
            .check()
            .unwrap();

        // Setup of ESB driver
        //
        // The BUFFER is consumed and split into two parts based on:
        // `app_to_radio_buf` (for sending packages to the driver)
        // `radio_to_app_buf` (for receiving packages from the driver)
        //
        // In the case of `app_to_radio_buf`, the buffer will initially be allocated (`grant`)
        // by the application, passed to the driver (`commit`) and eventually
        // freed (`release`) by the driver.
        // In the case of `radio_to_app_buf`, the driver will initially allocate the buffer
        // pass it to the application, which later needs to release the buffer.
        //
        // The `esb_app` is the returned resource (object) for the application, allowing
        // to read and send packages.
        //
        // The `esb_irq` is the returned resource (object) for the driver itself
        // On a `RADIO` interrupt we call the `radio_interrupt` method on `esb_irq` object,
        // which services the interrupt based on the "role" (PTX/PRX).
        //
        // The `esb_timer` is the returned resource (object) used to manage timing behavior
        // of the ESB driver.
        // On a `TIMER0` interrupt we call the `timer_interrupt` method on the `esb_irq` object.
        let (esb_app, esb_irq, esb_timer) = BUFFER
            .try_split(ctx.device.TIMER0, ctx.device.RADIO, addresses, config)
            .unwrap();

        // We set the "role" of the driver to PRX and start receiving packages.
        let mut esb_irq = esb_irq.into_prx();
        esb_irq.start_receiving().unwrap();

        // We pass ownership of the resources (objects) to RTIC, to be further used
        // by the tasks below.
        init::LateResources {
            esb_app,
            esb_irq,
            esb_timer,
        }
    }

    #[idle(resources = [esb_app])]
    fn idle(ctx: idle::Context) -> ! {
        let mut ct_rx: usize = 0;

        // Configure our package header for sending packages.
        // For this example all packages go over pipe 0.
        //
        // TODO: pid should be circulated (by the driver?)
        // TODO: the role of .no_ack for PRX role should be clarified
        // TODO: check should be clarified (what can go wrong here?)
        let esb_header = EsbHeader::build()
            .max_payload(MAX_PAYLOAD_SIZE)
            .pid(0)
            .pipe(0)
            .no_ack(false)
            .check()
            .unwrap();

        // Simple polling example
        // TODO: WFI operation (should be possible as incoming packages)
        // TODO: Defer incoming package to task allowing preemptive scheduling
        loop {
            // Dequeue received package (if any)
            if let Some(packet) = ctx.resources.esb_app.read_packet() {
                // internally of the driver the packet is first committed
                // before ownership is passed to the application.
                ct_rx += 1;
                let text = core::str::from_utf8(&packet[..]).unwrap();
                let rssi = packet.get_header().rssi();
                rprintln!(
                    "Received PKG #: {} | Payload: {} | rssi: {}",
                    ct_rx,
                    text,
                    rssi
                );
                // as we own the packet we can no free it (recycle it)
                packet.release();

                // Respond in the next transfer
                let mut response = ctx.resources.esb_app.grant_packet(esb_header).unwrap();
                let length = RESP.as_bytes().len();
                &response[..length].copy_from_slice(RESP.as_bytes());
                response.commit(length);
            }
        }
    }

    // UNCLEAR:
    // What is triggering the RADIO interrupt?
    // New transmission status, error/ok, or could it be something else?
    #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
    fn radio(ctx: radio::Context) {
        // UNCLEAR:
        // If in "ack" mode the `radio_interrupt` schedules the next timer interrupt for
        // for the actual "ack" (expecting there is an ack enqueued)?
        match ctx.resources.esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {
                rprintln!("Radio Interrupt MaximumAttempts");
            }
            Err(e) => panic!("Radio Interrupt Error {:?}", e),
            Ok(_) => {
                rprintln!("Radio Interrupt Ok");
            }
        }
    }

    #[task(binds = TIMER0, resources = [esb_timer], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        ctx.resources.esb_timer.timer_interrupt();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    rprintln!("{}", info);
    loop {
        compiler_fence(Ordering::SeqCst);
    }
}

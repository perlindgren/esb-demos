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

        static BUFFER: EsbBuffer<U1024, U1024> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };
        let addresses = Addresses::default();
        let config = ConfigBuilder::default()
            .maximum_transmit_attempts(1)
            .max_payload_size(MAX_PAYLOAD_SIZE)
            .check()
            .unwrap();
        let (esb_app, esb_irq, esb_timer) = BUFFER
            .try_split(ctx.device.TIMER0, ctx.device.RADIO, addresses, config)
            .unwrap();
        let mut esb_irq = esb_irq.into_prx();
        esb_irq.start_receiving().unwrap();

        init::LateResources {
            esb_app,
            esb_irq,
            esb_timer,
        }
    }

    #[idle(resources = [esb_app])]
    fn idle(ctx: idle::Context) -> ! {
        let mut ct_rx: usize = 0;
        let esb_header = EsbHeader::build()
            .max_payload(MAX_PAYLOAD_SIZE)
            .pid(0)
            .pipe(0)
            .no_ack(false)
            .check()
            .unwrap();
        loop {
            // Did we receive any packet ?
            if let Some(packet) = ctx.resources.esb_app.read_packet() {
                ct_rx += 1;
                let text = core::str::from_utf8(&packet[..]).unwrap();
                let rssi = packet.get_header().rssi();
                rprintln!(
                    "Received PKG #: {} | Payload: {} | rssi: {}",
                    ct_rx,
                    text,
                    rssi
                );
                packet.release();

                // Respond in the next transfer
                let mut response = ctx.resources.esb_app.grant_packet(esb_header).unwrap();
                let length = RESP.as_bytes().len();
                &response[..length].copy_from_slice(RESP.as_bytes());
                response.commit(length);
            }
        }
    }

    #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
    fn radio(ctx: radio::Context) {
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

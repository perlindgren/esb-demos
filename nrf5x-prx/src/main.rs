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
    embedded_hal::digital::v2::OutputPin,
    // nrf52832_hal::gpio;
    // use nrf52832_hal::gpio::p0::*;
    // use nrf52832_hal::gpio::Level;
    // use nrf52832_hal::gpio::*;
    // use nrf52832_hal::spim::Spim;
    esb::{
        consts::*, irq::StatePRX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
        EsbBuffer, EsbHeader, EsbIrq, IrqTimer,
    },
    hal::{
        gpio::{self, p0, p0::*, p1, p1::*, Level, PushPull},
        pac::TIMER0,
        spim::Spim,
    },
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
        let p = ctx.device;
        let _clocks = hal::clocks::Clocks::new(p.CLOCK).enable_ext_hfosc();
        rtt_init_print!();
        rprintln!("RTX init");

        let port0 = p0::Parts::new(p.P0);
        let port1 = p1::Parts::new(p.P1);

        let mut led1: P0_13<gpio::Output<PushPull>> =
            port0.p0_13.into_push_pull_output(Level::High);
        let mut led2: P0_14<gpio::Output<PushPull>> =
            port0.p0_14.into_push_pull_output(Level::High);
        let mut led3: P0_15<gpio::Output<PushPull>> =
            port0.p0_15.into_push_pull_output(Level::High);
        let mut led4: P0_16<gpio::Output<PushPull>> =
            port0.p0_16.into_push_pull_output(Level::High);

        let _ = led1.set_low();
        let _ = led2.set_low();
        let _ = led3.set_low();
        let _ = led4.set_low();
        // let _ = led1.set_high();
        // let _ = led2.set_high();
        // let _ = led3.set_high();
        // let _ = led4.set_high();

        let mut cs = port1.p1_01.into_push_pull_output(Level::High);
        let spimiso = port1.p1_02.into_floating_input().degrade();
        let spimosi = port1.p1_03.into_push_pull_output(Level::Low).degrade();
        let spiclk = port1.p1_04.into_push_pull_output(Level::Low).degrade();

        let pins = hal::spim::Pins {
            sck: spiclk,
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };

        let mut spi = Spim::new(
            p.SPIM2,
            pins,
            hal::spim::Frequency::K500,
            hal::spim::MODE_3,
            0,
        );

        //let _ = cs.set_high();

        // loop {
        //     let _ = cs.set_high();
        //     let _ = p25.set_high();
        //     rprintln!("high");
        //     let _ = cs.set_low();
        //     let _ = p25.set_low();
        //     rprintln!("low");
        // }

        // let mut tests_ok = true;

        // rprintln!("RTX init");
        // let reference_data = &[0u8, 1, 2, 3];

        // rprintln!("loopback {:?}", reference_data);

        // // Read only test vector
        // let test_vec1 = *reference_data;
        // rprintln!("test_vec1 {:?}", test_vec1);
        // rprintln!("{:p}", &test_vec1);
        // let mut readbuf = [0; 16];

        // This will write 8 bytes, then shift out ORC

        // Note :     spi.read( &mut cs.degrade(), reference_data, &mut readbuf )
        //            will fail because reference data is in flash, the copy to
        //            an array will move it to RAM.

        // match spi.read(&mut cs.degrade(), &test_vec1, &mut readbuf) {
        //     Ok(_) => {
        //         for i in 0..test_vec1.len() {
        //             tests_ok &= test_vec1[i] == readbuf[i];
        //         }
        //         if !tests_ok {
        //             led1.set_low().unwrap();
        //         } else {
        //             const ORC: u8 = 0;
        //             for i in test_vec1.len()..readbuf.len() {
        //                 if ORC != readbuf[i] {
        //                     tests_ok = false;
        //                     led1.set_low().unwrap();
        //                 }
        //             }
        //         }
        //     }
        //     Err(err) => {
        //         rprintln!("error {:?}", err);
        //         tests_ok = false;
        //         led1.set_low().unwrap();
        //     }
        // }

        // rprintln!("test {:?}, data back {:?}", tests_ok, readbuf);
        // let mut cs = cs.degrade();
        // //let who_am_i = &[0x0Fu8 << 1 | 1, 0]; // read bit set
        // let who_am_i = &[0x8Fu8, 0]; // read bit set
        //                              //rprintln!("req {:?}", req);
        // let mut first = true;
        // loop {
        //     let mut req = *who_am_i;
        //     match spi.transfer(&mut cs, &mut req) {
        //         Ok(_) => {
        //             if first {
        //                 first = false;
        //                 rprintln!("ok {:?}", req);
        //             }
        //         }
        //         Err(err) => {
        //             rprintln!("error {:?}", err);
        //         }
        //     };
        // }

        // We statically allocate space for buffers.
        static BUFFER: EsbBuffer<U1024, U1024> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };

        // ESB default pipe address mapping
        let addresses = Addresses::default();

        // ESB driver configuration
        // Default is to listen to packages on all pipes
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
            .try_split(p.TIMER0, p.RADIO, addresses, config)
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

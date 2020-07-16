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
        consts::*, irq::StatePTX, Addresses, BBBuffer, ConfigBuilder, ConstBBBuffer, Error, EsbApp,
        EsbBuffer, EsbHeader, EsbIrq, IrqTimer,
    },
    hal::{
        gpio::{self, p0, p0::*, Level, PushPull},
        pac::{TIMER0, TIMER1},
        spim::Spim,
    },
    rtt_target::{rprintln, rtt_init_print},
};

use heapless::String;

const MAX_PAYLOAD_SIZE: u8 = 64;

static DELAY_FLAG: AtomicBool = AtomicBool::new(false);
static ATTEMPTS_FLAG: AtomicBool = AtomicBool::new(false);

#[rtic::app(device = crate::hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        esb_app: EsbApp<U1024, U1024>,
        esb_irq: EsbIrq<U1024, U1024, TIMER0, StatePTX>,
        esb_timer: IrqTimer<TIMER0>,
        delay: TIMER1,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let d = ctx.device;

        let _clocks = hal::clocks::Clocks::new(d.CLOCK).enable_ext_hfosc();
        rtt_init_print!();
        rprintln!("PTX init");

        let port0 = p0::Parts::new(d.P0);

        // Who am I

        let mut cs = port0.p0_27.into_push_pull_output(Level::High);
        let spimiso = port0.p0_23.into_floating_input().degrade();
        let spimosi = port0.p0_29.into_push_pull_output(Level::Low).degrade();
        let spiclk = port0.p0_12.into_push_pull_output(Level::Low).degrade();

        let pins = hal::spim::Pins {
            sck: spiclk,
            miso: Some(spimiso),
            mosi: Some(spimosi),
        };

        let mut spi = Spim::new(
            d.SPIM2,
            pins,
            hal::spim::Frequency::K500,
            hal::spim::MODE_3,
            0,
        );

        // rprintln!("test {:?}, data back {:?}", tests_ok, readbuf);
        let mut cs = cs.degrade();
        //let who_am_i = &[0x0Fu8 << 1 | 1, 0]; // read bit set
        let who_am_i = &[0x8Fu8, 0]; // read bit set
                                     //rprintln!("req {:?}", req);
        let mut first = true;
        
            let mut req = *who_am_i;
            match spi.transfer(&mut cs, &mut req) {
                Ok(_) => {
                    if first {
                        first = false;
                        rprintln!("ok {:?}", req);
                    }
                }
                Err(err) => {
                    rprintln!("error {:?}", err);
                }
            };
        

       

        static BUFFER: EsbBuffer<U1024, U1024> = EsbBuffer {
            app_to_radio_buf: BBBuffer(ConstBBBuffer::new()),
            radio_to_app_buf: BBBuffer(ConstBBBuffer::new()),
            timer_flag: AtomicBool::new(false),
        };
        let addresses = Addresses::default();
        let config = ConfigBuilder::default()
            .maximum_transmit_attempts(0)
            .max_payload_size(MAX_PAYLOAD_SIZE)
            .check()
            .unwrap();

        let (esb_app, esb_irq, esb_timer) = BUFFER
            .try_split(d.TIMER0, d.RADIO, addresses, config)
            .unwrap();

        // setup timer for delay
        let timer = d.TIMER1;
        timer.bitmode.write(|w| w.bitmode()._32bit());

        // 16 Mhz / 2**9 = 31250 Hz
        timer.prescaler.write(|w| unsafe { w.prescaler().bits(9) });
        timer.shorts.modify(|_, w| w.compare0_clear().enabled());
        timer.cc[0].write(|w| unsafe { w.bits(31250u32) });
        timer.events_compare[0].reset();
        timer.intenset.write(|w| w.compare0().set());

        // Clears and starts the counter
        timer.tasks_clear.write(|w| unsafe { w.bits(1) });
        timer.tasks_start.write(|w| unsafe { w.bits(1) });

        init::LateResources {
            esb_app,
            esb_irq: esb_irq.into_ptx(),
            esb_timer,
            delay: timer,
        }
    }

    #[idle(resources = [esb_app])]
    fn idle(ctx: idle::Context) -> ! {
        let mut pid = 0;
        let mut ct_tx: usize = 0;
        let mut ct_rx: usize = 0;
        let mut ct_err: usize = 0;
        loop {
            let esb_header = EsbHeader::build()
                .max_payload(MAX_PAYLOAD_SIZE)
                .pid(pid)
                .pipe(0)
                .no_ack(false)
                .check()
                .unwrap();
            if pid == 3 {
                pid = 0;
            } else {
                pid += 1;
            }

            // Did we receive any packet ?
            if let Some(response) = ctx.resources.esb_app.read_packet() {
                ct_rx += 1;
                let text = core::str::from_utf8(&response[..]).unwrap();
                let rssi = response.get_header().rssi();
                rprintln!(
                    "Received PKG #: {} | Payload: {} | rssi: {}",
                    ct_rx,
                    text,
                    rssi
                );
                response.release();
            }
            ct_tx += 1;
            rprintln!("Sending PKG | tx: {} | err: {}", ct_tx, ct_err);

            let mut packet = ctx.resources.esb_app.grant_packet(esb_header).unwrap();
            let mut s: String<U16> = String::new();
            write!(&mut s, "PTX PKG #{}", ct_tx).unwrap();
            let length = s.as_bytes().len();
            &packet[..length].copy_from_slice(s.as_bytes());
            packet.commit(length);
            ctx.resources.esb_app.start_tx();

            while !DELAY_FLAG.load(Ordering::Acquire) {
                if ATTEMPTS_FLAG.load(Ordering::Acquire) {
                    rprintln!("--- Ack not received {}\r", ct_err);
                    ATTEMPTS_FLAG.store(false, Ordering::Release);
                    ct_err += 1;
                }
            }
            DELAY_FLAG.store(false, Ordering::Release);
        }
    }

    #[task(binds = RADIO, resources = [esb_irq], priority = 3)]
    fn radio(ctx: radio::Context) {
        match ctx.resources.esb_irq.radio_interrupt() {
            Err(Error::MaximumAttempts) => {
                rprintln!("Radio Interrupt MaximumAttempts");
                ATTEMPTS_FLAG.store(true, Ordering::Release);
            }
            Err(e) => panic!("Radio Interrupt Error {:?}", e),
            Ok(_) => rprintln!("Radio Interrupt Ok"),
        }
    }

    #[task(binds = TIMER0, resources = [esb_timer], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        ctx.resources.esb_timer.timer_interrupt();
    }

    #[task(binds = TIMER1, resources = [delay], priority = 1)]
    fn timer1(ctx: timer1::Context) {
        ctx.resources.delay.events_compare[0].reset();
        DELAY_FLAG.store(true, Ordering::Release);
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

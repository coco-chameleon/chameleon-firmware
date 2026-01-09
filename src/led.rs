use core::marker::PhantomData;
use esp_hal::gpio;
use esp_hal::gpio::Level;
use esp_hal::peripherals::{RMT, SYSTEM};
use esp_hal::rmt::PulseCode;

pub struct LedController<'d> {
    _phantom: PhantomData<&'d ()>,
}

impl<'d> LedController<'d> {
    pub fn new<'a>(
        gpio: impl gpio::interconnect::PeripheralOutput<'d>,
        _rmt: RMT<'d>
    ) -> Self {
        let pin: gpio::interconnect::OutputSignal = gpio.into();
        pin.apply_output_config(&gpio::OutputConfig::default());
        pin.set_output_enable(true);
        gpio::OutputSignal::RMT_SIG_0.connect_to(&pin);

        SYSTEM::regs().perip_clk_en0()
            .modify(|_, w| w.rmt_clk_en().set_bit());

        let reg = RMT::regs();

        reg.sys_conf().write(|w| unsafe {
            // set peripheral clock to 80 MHz
            w.sclk_sel().bits(1); // APB_CLK
            w.sclk_div_num().bits(0); // no division
            w.sclk_active().set_bit()
        });

        reg.ch0_tx_conf0().write(|w| unsafe {
            // set channel clock to 10 MHz (0.1 us period)
            w.div_cnt().bits(8);

            // don't apply a carrier
            w.carrier_en().clear_bit();

            // use all 384 words of available RMT RAM
            // this disables all other channels
            w.mem_size().bits(8)
        });

        LedController { _phantom: PhantomData }
    }

    fn write(&self, value: PulseCode) {
        RMT::regs().ch0data()
            .write(|w| unsafe { w.bits(value.into()) });
    }

    fn encode(&self, color: u32) {
        let one = PulseCode::new(Level::High, 9, Level::Low, 3);
        let zero = PulseCode::new(Level::High, 3, Level::Low, 9);
        let mut value = color;
        // green - value starts as xxRRGGBB
        for _ in 0..8 {
            self.write(if value & 0x8000 != 0 { one } else { zero });
            value = value << 1;
        }
        // red - value is now <<8 so RRGGBBxx
        for _ in 0..8 {
            self.write(if value & 0x8000_0000 != 0 { one } else { zero });
            value = value << 1;
        }
        // blue - value is now <<16 so GGBBxxxx
        for _ in 0..8 {
            self.write(if value & 0x80_0000 != 0 { one } else { zero });
            value = value << 1;
        }
    }

    fn delay(&self, centiseconds: u32) {
        // approximate the requested delay with max-length symbols
        // the maximum error is half the symbol length, about 3 ms
        let mut delay = centiseconds * 100_000; // 10 ms -> 0.1 us
        while delay > PulseCode::MAX_LEN as u32 {
            self.write(PulseCode::new(
                Level::Low, PulseCode::MAX_LEN,
                Level::Low, PulseCode::MAX_LEN,
            ));
            delay -= PulseCode::MAX_LEN as u32 * 2;
        }
    }

    fn end(&self) {
        self.write(PulseCode::end_marker());
    }

    fn reset(&self) {
        RMT::regs().ch0_tx_conf0().modify(|_, w| {
            w.tx_stop().set_bit();
            w.apb_mem_rst().set_bit();
            w.conf_update().set_bit()
        });
    }

    fn start(&self, continuous: bool) {
        RMT::regs().ch0_tx_conf0().modify(|_, w| {
            w.tx_conti_mode().bit(continuous);
            w.conf_update().set_bit();
            w.mem_rd_rst().set_bit();
            w.tx_start().set_bit()
        });
    }

    pub fn solid(&mut self, color: u32) {
        self.reset();

        self.encode(color);
        self.end();

        self.start(false);
    }

    pub fn animate(&mut self, steps: &[u32]) {
        self.reset();

        for step in steps {
            self.encode(*step);
            self.delay(*step >> 24);
        }
        self.end();

        self.start(true);
    }
}

#![no_main]
#![no_std]

use panic_rtt_target as _;

use async_embedded::task;
use core::cell::Cell;
use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll, Waker};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::stm32::{interrupt, Interrupt, NVIC};
use stm32f4xx_hal::timer::{Event, Timer};

struct InterruptObject {
    nr: Interrupt,
    waker: Mutex<Cell<Option<Waker>>>,
    taken: AtomicBool,
}

impl InterruptObject {
    pub const fn new(nr: Interrupt) -> Self {
        Self {
            nr,
            waker: Mutex::new(Cell::new(None)),
            taken: AtomicBool::new(false),
        }
    }

    pub fn handle_interrupt(&self) {
        NVIC::mask(self.nr);

        // We are already in the interrupt context, construct cs in a dirty way
        let cs = unsafe { core::mem::transmute(()) };
        if let Some(waker) = self.waker.borrow(&cs).take() {
            waker.wake();
        }
        cortex_m::asm::sev();
    }

    fn arm(&self, waker: Waker) {
        cortex_m::interrupt::free(|cs| {
            self.waker.borrow(cs).set(Some(waker));
        })
    }

    pub fn get_handle(&'static self) -> Option<InterruptHandle> {
        if self.taken.swap(false, Ordering::SeqCst) {
            None
        } else {
            Some(InterruptHandle { obj: self })
        }
    }
}

struct InterruptHandle {
    obj: &'static InterruptObject,
}

impl InterruptHandle {
    pub async fn wait(&mut self) {
        self.await
    }

    pub fn unpend(&self) {
        NVIC::unpend(self.obj.nr)
    }
}

impl Future for InterruptHandle {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let obj = self.get_mut().obj;
        if NVIC::is_pending(obj.nr) {
            Poll::Ready(())
        } else {
            obj.arm(cx.waker().clone());
            unsafe { NVIC::unmask(obj.nr) };
            Poll::Pending
        }
    }
}

static TIM2_OBJ: InterruptObject = InterruptObject::new(Interrupt::TIM2);

#[interrupt]
fn TIM2() {
    TIM2_OBJ.handle_interrupt();

    rprintln!("ISR");
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = stm32::Peripherals::take().unwrap();

    // Apply sleep-mode workaround to enable RTT connection
    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });
    dp.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

    let rcc = dp.RCC.constrain();
    //let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(100.mhz()).freeze();
    let clocks = rcc.cfgr.sysclk(16.mhz()).pclk1(8.mhz()).freeze();

    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_open_drain_output();
    led.set_high().ok();

    let mut tim2_irq = TIM2_OBJ.get_handle().unwrap();

    let mut timer = Timer::tim2(dp.TIM2, 1.hz(), clocks);
    timer.listen(Event::TimeOut);
    unsafe { NVIC::unmask(Interrupt::TIM2) };

    rprintln!("init");
    task::block_on(async {
        loop {
            rprintln!("before await");
            tim2_irq.wait().await;
            rprintln!("after await");

            // Clear pending interrupt event
            if timer.wait().is_ok() {
                // Optionally we need this to synchronize with the slower bus
                //cortex_m::asm::delay(16);

                // At this point TIM2 IRQ should be de-asserted. If we unpend
                // the IRQ, it will not become pending instantly.
                tim2_irq.unpend();

                rprintln!("tim2_int handled");
                led.toggle().ok();
            }
        }
    })
}

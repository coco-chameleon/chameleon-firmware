// SPDX-FileCopyrightText: 2026 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::rng::Rng;
use esp_radio::wifi::{ClientConfig, ModeConfig, ScanConfig, WifiController, WifiDevice, WifiEvent, WifiStaState};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");


pub async fn new(
    spawner: Spawner,
    device: esp_hal::peripherals::WIFI<'static>,
    radio_controller: &'static esp_radio::Controller<'static>,
) {
    let (wifi_controller, wifi_interfaces) =
        esp_radio::wifi::new(&radio_controller, device, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let dhcp_config = embassy_net::Config::dhcpv4(Default::default());

    let rng = Rng::new();

    let (_net_stack, net_runner) = embassy_net::new(
        wifi_interfaces.sta,
        dhcp_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        (rng.random() as u64) << 32 | rng.random() as u64,
    );

    spawner.spawn(wifi_connection_task(wifi_controller)).ok();
    spawner.spawn(net_task(net_runner)).ok();
}

#[embassy_executor::task]
async fn wifi_connection_task(mut controller: WifiController<'static>) {
    loop {
        if esp_radio::wifi::sta_state() == WifiStaState::Connected {
            // wait until we're no longer connected
            controller
                .wait_for_event(WifiEvent::StaDisconnected)
                .await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let station_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(WIFI_SSID.into())
                    .with_password(WIFI_PSK.into()),
            );
            controller.set_config(&station_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");

            info!("Scan");
            let scan_config = ScanConfig::default();
            let result = controller
                .scan_with_config_async(scan_config)
                .await
                .unwrap();
            for ap in result {
                info!("{:?}", ap);
            }
        }
        info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => info!("Wifi connected!"),
            Err(e) => {
                error!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

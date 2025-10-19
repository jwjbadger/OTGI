#![allow(clippy::uninlined_format_args)]

use esp_idf_hal::{delay::FreeRtos, gpio, prelude::*, timer::*};
use esp_idf_svc::{
    bt::{
        ble::{
            gap::EspBleGap,
            gatt::{server::EspGatts, Permission, Property},
        },
        Ble, BtDriver, BtUuid,
    },
    nvs::{self, EspDefaultNvsPartition},
};
use otgi::{obd, wireless};
use std::sync::Arc;

const SERVICE_UUID: u128 = 0x2cbc6002370f577a928681e04f368400;
const FUEL_USAGE_CHARACTERISTIC_UUID: u128 = 0x56c46fef90390803a71feebcc8650e43;
const RUNCOUNT_CHARACTERISTIC_UUID: u128 = 0xed0cdaa9fc55c2c193a061b6e1f36720;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let nvs_partition = EspDefaultNvsPartition::take().unwrap();
    let bt =
        Arc::new(BtDriver::<Ble>::new(peripherals.modem, Some(nvs_partition.clone())).unwrap());
    let pins = peripherals.pins;

    let nvs_namespace = match nvs::EspNvs::new(nvs_partition, "otgi_data", true) {
        Ok(nvs) => nvs,
        Err(e) => panic!("Could't get namespace {e:?}"),
    };

    let runcount: u64 = if !nvs_namespace.contains("runcount").unwrap() {
        nvs_namespace.set_u64("runcount", 0).unwrap();
        0
    } else {
        let rc = nvs_namespace.get_u64("runcount").unwrap().unwrap();
        nvs_namespace.set_u64("runcount", rc + 1).unwrap();
        rc + 1
    };

    let mut liters_used: f64 = 0.0;

    let fuel_usage_uuid = BtUuid::uuid128(FUEL_USAGE_CHARACTERISTIC_UUID);
    let runcount_uuid = BtUuid::uuid128(RUNCOUNT_CHARACTERISTIC_UUID);
    let ble_server = wireless::Server::new(
        Arc::new(EspBleGap::new(bt.clone()).unwrap()),
        Arc::new(EspGatts::new(bt.clone()).unwrap()),
        wireless::ServerConfiguration {
            services: vec![wireless::ServiceDescriptor {
                uuid: BtUuid::uuid128(SERVICE_UUID),
                is_primary: true,
                characteristics: vec![
                    wireless::CharacteristicDescriptor {
                        uuid: fuel_usage_uuid.clone(),
                        permissions: Permission::Write | Permission::Read,
                        properties: Property::Indicate.into(),
                        max_len: 200,
                        data: liters_used.to_le_bytes().to_vec(),
                    },
                    wireless::CharacteristicDescriptor {
                        uuid: runcount_uuid.clone(),
                        permissions: Permission::Write | Permission::Read,
                        properties: Property::Indicate | Property::Read,
                        max_len: 200,
                        data: runcount.to_le_bytes().to_vec(),
                    },
                ],
            }],
            name: "OTGI",
        },
    );

    let gap_server = ble_server.clone();
    ble_server
        .gap
        .subscribe(move |event| {
            gap_server.handle_gap_event(event);
        })
        .unwrap();

    let gatts_server = ble_server.clone();
    ble_server
        .gatts
        .subscribe(move |(gatt_intf, event)| {
            gatts_server.handle_gatts_event(gatt_intf, event);
        })
        .unwrap();

    ble_server.gatts.register_app(wireless::APP_ID).unwrap();

    log::info!("BLE Gap and Gatts initialized");

    let mut high_ref = gpio::PinDriver::output(pins.gpio25).unwrap();
    high_ref.set_high().unwrap();

    let mut driver = obd::ObdDriver::try_new(
        peripherals.can,
        pins.gpio33,
        pins.gpio32,
        &Default::default(),
    )
    .unwrap();
    driver.start().expect("Failed to start driver");

    let mut timer = TimerDriver::new(peripherals.timer00, &TimerConfig::new()).unwrap();
    let timer_hz = timer.tick_hz() as f64;

    let mut ltft = 0.0;
    let mut stft = 0.0;

    let mut stft_last_updated = 0.0;
    let mut ltft_last_updated = 0.0;

    let mut fuel_usage_last_updated = 0.0;

    let mut timer_enabled = false;

    let stft_query = obd::ObdQuery::new(
        obd::ObdMode::QueryNow,
        Some(obd::PID::ShortTermFuelTrimBankOne),
    );
    let ltft_query = obd::ObdQuery::new(
        obd::ObdMode::QueryNow,
        Some(obd::PID::ShortTermFuelTrimBankOne),
    );
    let maf_query = obd::ObdQuery::new(
        obd::ObdMode::QueryNow,
        Some(obd::PID::ShortTermFuelTrimBankOne),
    );

    loop {
        let mut time = timer.counter().unwrap() as f64 / timer_hz;
        // Update stft at 5 Hz
        if timer_enabled && time > stft_last_updated + 0.2 {
            if let Ok(obd::ObdReadableData::SignedPercentage(stft_res)) = driver.query(&stft_query)
            {
                stft = stft_res;
                log::info!("Updated stft: {:?}", stft);
                stft_last_updated = time;
                FreeRtos::delay_ms(50);
                time = timer.counter().unwrap() as f64 / timer_hz;
            }
        }

        // Update ltft at 1 Hz
        if timer_enabled && time > ltft_last_updated + 1.0 {
            if let Ok(obd::ObdReadableData::SignedPercentage(ltft_res)) = driver.query(&ltft_query)
            {
                ltft = ltft_res;
                log::info!("Updated ltft");
                ltft_last_updated = time;
                FreeRtos::delay_ms(50);
            }
        }

        let maf = driver.query(&maf_query);
        FreeRtos::delay_ms(50);
        time = timer.counter().unwrap() as f64 / timer_hz;

        if let Ok(obd::ObdReadableData::Raw(maf)) = maf {
            // Only start timer once data is being read to avoid assuming a massive fuel usage if
            // the esp is booted before the car
            if !timer_enabled {
                timer_enabled = true;
                timer.enable(true).unwrap();

                // Read diagnostic codes on startup
                let codes = driver
                    .query(&obd::ObdQuery::new(obd::ObdMode::QueryDTC, None))
                    .expect("couldn't read DTC");
                log::info!("Read DTC codes: {:#?}", codes);
                FreeRtos::delay_ms(50);
                time = timer.counter().unwrap() as f64 / timer_hz;
            }

            let usage = (maf * 3600.0) / ((14.7 * (1.0 + ((stft + ltft) / 100.0))) * 740.0);
            liters_used += f64::from(usage / 3600.0) * (time - fuel_usage_last_updated);
            fuel_usage_last_updated = time;
        } else if timer_enabled {
            // If the car is turned off and then back on, we should restart the timer
            stft_last_updated = 0.0;
            ltft_last_updated = 0.0;
            fuel_usage_last_updated = 0.0;
            timer.enable(false).unwrap();
            timer_enabled = false;
        }

        ble_server
            .indicate(&fuel_usage_uuid, &liters_used.to_le_bytes())
            .unwrap();
    }
}

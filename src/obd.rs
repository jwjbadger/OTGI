use esp_idf_hal::{
    can, delay,
    gpio::{InputPin, OutputPin},
    peripheral::Peripheral,
    sys::EspError,
};

// TODO: more idiomatic
fn human_readable(data: &[u8], format: PID) -> Result<f32, ()> {
    match format {
        PID::MassAirFlow => {
            if data.len() < 2 {
                return Err(());
            }
            Ok((256.0 * (data[0] as f32) + (data[1] as f32)) / 100.0)
        } // (256A + B) / 100
        PID::EngineFuelRate => {
            if data.len() < 2 {
                return Err(());
            }
            Ok((256.0 * (data[0] as f32) + (data[1] as f32)) / 20.0)
        } // (256A + B) / 20
        PID::EngineSpeed => {
            if data.len() < 2 {
                return Err(());
            }
            Ok((256.0 * (data[0] as f32) + (data[1] as f32)) / 4.0)
        } // (256A + B)/4
        PID::RunTime => {
            if data.len() < 2 {
                return Err(());
            }
            Ok(256.0 * (data[0] as f32) + (data[1] as f32))
        } // 256A + B
        PID::VehicleSpeed => {
            if data.is_empty() {
                return Err(());
            }
            Ok(data[0] as f32)
        }
        PID::ThrottlePosition | PID::FuelTankLevelInput | PID::RelativeThrottlePosition => {
            if data.is_empty() {
                return Err(());
            }
            Ok((data[0] as f32) / 2.55)
        } // A/2.55
        PID::Odometer => {
            if data.len() < 4 {
                return Err(());
            }
            Ok(((data[0] as f32) * 2.0_f32.powi(24)
                + (data[1] as f32) * 2.0_f32.powi(16)
                + (data[2] as f32) * 2.0_f32.powi(8)
                + (data[3] as f32))
                / 10.0_f32)
        } // (A(2^24) + B (2^16) + C (2^8) + D) / 10
        PID::ShortTermFuelTrimBankOne | PID::LongTermFuelTrimBankOne => {
            if data.is_empty() {
                return Err(());
            }
            Ok(((data[0] as f32) / 1.28) - 100.0)
        }
        _ => Err(()),
    }
}

#[repr(u8)]
#[derive(strum::FromRepr, Copy, Clone, Debug)]
pub enum PID {
    FirstCap = 0x00,
    EngineSpeed = 0x0C,
    VehicleSpeed = 0x0D,
    ThrottlePosition = 0x11,
    RunTime = 0x1F,
    SecondCap = 0x20,
    FuelTankLevelInput = 0x2F,
    RelativeThrottlePosition = 0x45,
    EngineFuelRate = 0x5E,
    Odometer = 0xA6,
    MassAirFlow = 0x10,
    ShortTermFuelTrimBankOne = 0x06,
    LongTermFuelTrimBankOne = 0x07,
    // TODO: additional PID's
}

impl From<PID> for u8 {
    fn from(pid: PID) -> Self {
        pid as u8
    }
}

#[derive(Debug, Clone)]
pub enum ObdError {
    Esp(EspError),
    MalformedResponse,
}

impl From<EspError> for ObdError {
    fn from(err: EspError) -> Self {
        Self::Esp(err)
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum ObdMode {
    QueryNow = 0x01,
    QueryFreezeFrame = 0x02,
    QueryDTC = 0x03,
    ClearDTC = 0x04,
    // TODO: additional modes
}

struct ObdRequest<'a> {
    mode: ObdMode,
    data: &'a [PID],
}

impl<'a> ObdRequest<'a> {
    fn assemble(&self) -> Result<[u8; 8], ()> {
        let bytes_follow = self.data.len() + 1;

        if bytes_follow > 0x07 {
            return Err(());
        }

        // TODO: make assembly more efficient
        let mut msg: [u8; 8] = [0; 8];
        msg[0] = bytes_follow as u8;
        msg[1] = self.mode as u8;
        msg[2..2 + self.data.len()].copy_from_slice(
            &self
                .data
                .iter()
                .cloned()
                .map(|e| e as u8)
                .collect::<Vec<u8>>(),
        );

        Ok(msg)
    }
}

pub struct ObdDriverConfig {
    timing: can::config::Timing,
}

impl Default for ObdDriverConfig {
    fn default() -> Self {
        Self {
            timing: can::config::Timing::B500K,
        }
    }
}

pub struct ObdDriver<'a> {
    can_driver: can::CanDriver<'a>,
}

impl<'a> ObdDriver<'a> {
    pub fn try_new(
        can: impl Peripheral<P = can::CAN> + 'a,
        tx: impl Peripheral<P = impl OutputPin> + 'a,
        rx: impl Peripheral<P = impl InputPin> + 'a,
        config: &ObdDriverConfig,
    ) -> Result<Self, ObdError> {
        Ok(Self {
            can_driver: can::CanDriver::new(
                can,
                tx,
                rx,
                &can::config::Config::new()
                    .filter(can::config::Filter::Standard {
                        filter: 0x7E0,
                        mask: 0xFF0,
                    })
                    .timing(config.timing),
            )?,
        })
    }

    pub fn start(&mut self) -> Result<(), ObdError> {
        Ok(self.can_driver.start()?)
    }

    pub fn query(&mut self, pid: PID) -> Result<f32, ObdError> {
        let tx_frame = can::Frame::new(
            0x7df,
            can::Flags::None.into(),
            &(ObdRequest {
                mode: ObdMode::QueryNow,
                data: &[pid],
            })
            .assemble()
            .unwrap(),
        )
        .unwrap();

        self.can_driver
            .transmit(&tx_frame, delay::TickType::new_millis(100).into())?;

        let rx_frame = self
            .can_driver
            .receive(delay::TickType::new_millis(100).into())?;

        if rx_frame.data()[1..3] != [0x40 + ObdMode::QueryNow as u8, pid as u8] {
            log::error!("Picked up the wrong packet: {:?}", rx_frame.data());
            return Err(ObdError::MalformedResponse);
        }

        let data_len = rx_frame.data()[0] - 2;

        if data_len < 1 {
            log::error!("Recieved empty packet: {:?}", rx_frame.data());
            return Err(ObdError::MalformedResponse);
        }

        let data = &rx_frame.data()[3..3 + (data_len as usize)];
        log::warn!("{:?}", data);

        // TODO: needs to handle multiple modes; make more idiomatic
        match human_readable(data, pid) {
            Ok(res) => Ok(res),
            Err(()) => Err(ObdError::MalformedResponse),
        }
    }

    // TODO: attempt to send multiple packets
}

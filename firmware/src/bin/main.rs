#![no_std]
#![no_main]

use esp_drv8833::drv8833::MotorLink;
use esp_drv8833::MotorTimer;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc};
use esp_hal::time::{Instant, Rate};
use esp_hal::{main, Blocking};
use esp_println::println;
use firmware::{BalanceControl, BalanceRobot};
use mpu6050_dmp::accel::{Accel, AccelFullScale};
use mpu6050_dmp::address::Address;
// use mpu6050_dmp::calibration::{CalibrationParameters, ReferenceGravity};
use mpu6050_dmp::error::Error;
use mpu6050_dmp::gyro::{Gyro, GyroFullScale};
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Panic! Msg: {}:{:?}", info.message(), info.location());
    loop {}
}

// First pair of values: (Accel { x: -52, y: 36, z: 16384 }, Gyro { x: 7, y: 7, z: 8 })

#[main]
fn main() -> ! {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let motor_config = MotorTimer::new(
        &ledc,
        timer::Number::Timer0,
        timer::config::Duty::Duty7Bit,
        Rate::from_hz(500),
    )
    .unwrap();

    let mut robot = BalanceRobot::new(
        &ledc,
        &motor_config.timer,
        MotorLink::new(channel::Number::Channel1, peripherals.GPIO0),
        MotorLink::new(channel::Number::Channel0, peripherals.GPIO1),
        MotorLink::new(channel::Number::Channel3, peripherals.GPIO10),
        MotorLink::new(channel::Number::Channel2, peripherals.GPIO9),
    )
    .unwrap();
    robot.brake().unwrap();

    let mut delay = Delay::new();
    delay.delay_millis(1000);

    // Setup correctly the accelerometer

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO4)
        .with_scl(peripherals.GPIO3);

    let mut sensor: Mpu6050<I2c<'_, esp_hal::Blocking>> =
        Mpu6050::new(i2c, Address::default()).unwrap();

    let offsets = (Accel::new(-2656, -1231, 797), Gyro::new(181, -9, 26));

    sensor.initialize_dmp(&mut delay).unwrap();
    sensor.set_accel_full_scale(AccelFullScale::G2).unwrap();
    sensor.set_gyro_full_scale(GyroFullScale::Deg250).unwrap();
    sensor.set_accel_calibration(&offsets.0).unwrap();
    sensor.set_gyro_calibration(&offsets.1).unwrap();

    // let values = sensor
    //     .calibrate(
    //         &mut delay,
    //         &CalibrationParameters::new(
    //             AccelFullScale::G2,
    //             GyroFullScale::Deg250,
    //             ReferenceGravity::ZP,
    //         ),
    //     )
    //     .unwrap();

    // println!("Calibration: {:?}", values);

    let temp = sensor.temperature().unwrap();
    println!("Temperature: {}°C", temp.celsius());

    sensor.reset_fifo().unwrap();

    const PI: f32 = 3.141592653589793;
    let mut control = BalanceControl::new(0.00, 800.0, 0.0, 3500.0);
    let mut pitch: f32 = 0.0;
    let mut last = 0;

    loop {
        let count = match sensor.get_fifo_count() {
            Ok(count) => count,
            Err(_) => continue,
        };

        if count >= 1024 {
            panic!("FIFO is full!");
        }

        // Sample rate of around ~5ms!
        if count >= 28 {
            pitch = match get_pitch(&mut sensor) {
                Ok(pitch) => pitch,
                Err(_) => continue,
            };
        }

        let dt = Instant::now().duration_since_epoch().as_micros() - last;
        if dt >= 5_000 {
            let (duty, error, p, i, d) = control.calc_duty(pitch);
            last = Instant::now().duration_since_epoch().as_micros();
            let angle = pitch * 180.0 / PI;

            match drive_robot(&mut robot, angle, duty) {
                Ok(_) => println!("{},{},{},{},{},{}", duty, angle, error, p, i, d),
                Err(_) => continue,
            }
        }
    }
}

fn get_pitch<'a>(sensor: &mut Mpu6050<I2c<'a, Blocking>>) -> Result<f32, Error<I2c<'a, Blocking>>> {
    let mut buf: [u8; 28] = [0; 28];
    let fifo = sensor.read_fifo(&mut buf)?;
    sensor.reset_fifo()?;

    let q = Quaternion::from_bytes(&fifo[..16])
        .ok_or(Error::WrongDevice)?
        .normalize();
    Ok(YawPitchRoll::from(q).pitch)
}

fn drive_robot(
    robot: &mut BalanceRobot,
    angle: f32,
    duty: i8,
) -> Result<(), esp_drv8833::drv8833::Error> {
    if angle > 30.0 || angle < -30.0 {
        robot.brake()?;
    } else {
        let value = duty.abs() as u8;
        if duty > 0 {
            robot.backwards(value)?;
        } else {
            robot.forward(value)?;
        }
    }

    Ok(())
}

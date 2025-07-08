#![no_std]

use esp_drv8833::{
    drv8833::{Error, MotorLink},
    Motor, MotorInterface, MotorSlowDecay,
};
use esp_hal::{
    gpio::interconnect::PeripheralOutput,
    ledc::{timer::Timer, Ledc, LowSpeed},
};

pub struct BalanceRobot<'a> {
    motor_right: MotorSlowDecay<'a>,
    motor_left: MotorSlowDecay<'a>,
}

impl<'a> BalanceRobot<'a> {
    pub fn new<A, B, C, D>(
        ledc: &'a Ledc<'a>,
        timer: &'a Timer<'a, LowSpeed>,
        m_r_a: MotorLink<A>,
        m_r_b: MotorLink<B>,
        m_l_a: MotorLink<C>,
        m_l_b: MotorLink<D>,
    ) -> Result<Self, Error>
    where
        A: for<'any> PeripheralOutput<'any>,
        B: for<'any> PeripheralOutput<'any>,
        C: for<'any> PeripheralOutput<'any>,
        D: for<'any> PeripheralOutput<'any>,
    {
        // Create the right motor
        let motor_right: MotorSlowDecay = Motor::new(ledc, timer, m_r_a, m_r_b)?;
        // Create the left motor
        let motor_left: MotorSlowDecay = Motor::new(ledc, timer, m_l_a, m_l_b)?;

        Ok(Self {
            motor_right,
            motor_left,
        })
    }

    pub fn brake(&self) -> Result<(), Error> {
        self.motor_left.brake()?;
        self.motor_right.brake()?;

        Ok(())
    }

    pub fn forward(&mut self, duty: u8) -> Result<(), Error> {
        self.motor_left.forward(duty)?;
        self.motor_right.forward(duty)?;

        Ok(())
    }

    pub fn backwards(&mut self, duty: u8) -> Result<(), Error> {
        self.motor_left.backward(duty)?;
        self.motor_right.backward(duty)?;

        Ok(())
    }

    pub fn turn_right(&mut self, duty: u8) -> Result<(), Error> {
        self.motor_left.forward(duty)?;
        self.motor_right.backward(duty)?;

        Ok(())
    }

    pub fn turn_left(&mut self, duty: u8) -> Result<(), Error> {
        self.motor_left.backward(duty)?;
        self.motor_right.forward(duty)?;

        Ok(())
    }
}

#[derive(Default)]
pub struct BalanceControl {
    setpoint: f32, // in degrees
    kp: f32,
    ki: f32,
    kd: f32,
    error_sum: f32,
    previous_input: f32,
}

impl BalanceControl {
    pub fn new(setpoint: f32, kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            setpoint,
            kp,
            ki,
            kd,
            error_sum: f32::default(),
            previous_input: f32::default(),
        }
    }

    pub fn calc_duty(&mut self, input: f32) -> (i8, f32, f32, f32, f32) {
        let error = self.setpoint - input;
        let d_input = input - self.previous_input;

        self.previous_input = input;

        let proportional = self.kp * error;
        self.error_sum = self.error_sum.clamp(-10.0, 10.0) + error;

        let integral = self.ki * self.error_sum;
        let derivative = self.kd * d_input;

        let out = proportional + integral - derivative;

        // Motor deadband.
        // if out > 0.0 {
        //     out += 10.0;
        // }

        // if out < 0.0 {
        //     out -= 10.0;
        // }

        return (
            out.clamp(-100.0, 100.0) as i8,
            error,
            proportional,
            integral,
            derivative,
        );
    }
}

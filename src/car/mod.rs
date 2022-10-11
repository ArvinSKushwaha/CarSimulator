use std::{ops::{AddAssign, Mul, Add}, f64::consts::PI};

use eframe::egui::plot::Polygon;

const CAR_1_PARAMS: CarParams = CarParams {
    l: 4.298,
    w: 1.674,
    m: 1.225,
    i_z: 1.538,
    l_f: 0.883,
    l_r: 1.508,
    h_cg: 0.557,
    c_sf: 20.89,
    c_sr: 20.89,
    mu: 1.048,
};
const GRAVITATIONAL_ACCELERATION: f64 = 9.81;

pub struct Car {
    pub state: CarState,
    control: CarControl,
    params: CarParams,
    pub time: f64,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            state: CarState {
                x: 0.0,
                y: 0.0,
                delta: 0.0,
                v: 0.0,
                phi: PI / 2.0,
                phidot: 0.0,
                beta: 0.0,
            },
            control: CarControl {
                a: 0.0,
                ddelta: 0.0,
            },
            params: CAR_1_PARAMS,
            time: 0.0,
        }
    }
}

#[derive(Copy, Clone)]
pub struct CarState {
    pub x: f64,
    pub y: f64,
    pub delta: f64,
    pub v: f64,
    pub phi: f64,
    pub phidot: f64,
    pub beta: f64,
}

impl Mul<f64> for CarState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            delta: self.delta * rhs,
            v: self.v * rhs,
            phi: self.phi * rhs,
            phidot: self.phidot * rhs,
            beta: self.beta * rhs,
        }
    }
}

impl Add<Self> for CarState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            delta: self.delta + rhs.delta,
            v: self.v + rhs.v,
            phi: self.phi + rhs.phi,
            phidot: self.phidot + rhs.phidot,
            beta: self.beta + rhs.beta,
        }
    }
}

impl AddAssign for CarState {
    fn add_assign(&mut self, rhs: Self) {
        *self = Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            delta: self.delta + rhs.delta,
            v: self.v + rhs.v,
            phi: self.phi + rhs.phi,
            phidot: self.phidot + rhs.phidot,
            beta: self.beta + rhs.beta,
        };
    }
}

#[derive(Copy, Clone)]
pub struct CarControl {
    pub a: f64,
    pub ddelta: f64,
}

#[derive(Copy, Clone)]
pub struct CarParams {
    l: f64,
    w: f64,
    m: f64,
    i_z: f64,
    l_f: f64,
    l_r: f64,
    h_cg: f64,
    c_sf: f64,
    c_sr: f64,
    mu: f64,
}

fn compute_derivatives(state: &CarState, params: &CarParams, control: &CarControl) -> CarState {
    let CarParams {
        l,
        w: _,
        m,
        i_z,
        l_f,
        l_r,
        h_cg,
        c_sf,
        c_sr,
        mu,
    } = params;
    let CarState {
        x: _,
        y: _,
        delta,
        v,
        phi,
        phidot,
        beta,
    } = state;
    let CarControl { a, ddelta } = control;
    let l_wb = l_f + l_r;

    let g = GRAVITATIONAL_ACCELERATION;

    if v.abs() <= 0.1 {
        let dx = v * (phi + beta).cos();
        let dy = v * (phi + beta).sin();
        let ddelta = ddelta;
        let dv = a;
        let dphi = v * beta.cos() * delta.tan() / l_wb;
        let dbeta = (1.0 + (delta.tan() * l / l_wb).powi(2)).recip()
            * (l_r / (l_wb * delta.cos().powi(2)) * ddelta);
        let dphidot = l_wb.recip()
            * (a * beta.cos() * delta.tan() - v * beta.sin() * delta.tan() * dbeta
                + v * beta.cos() / delta.cos().powi(2) * ddelta);

        CarState {
            x: dx,
            y: dy,
            delta: *ddelta,
            v: *dv,
            phi: dphi,
            phidot: dphidot,
            beta: dbeta,
        }
    } else {
        let dx = v * (phi + beta).cos();
        let dy = v * (phi + beta).sin();
        let ddelta = ddelta;
        let dv = a;
        let dphi = phidot;
        let dphidot = mu * m / (i_z * l_wb)
            * (l_f * c_sf * (g * l_r - a * h_cg) * delta
                + (l_r * c_sr * (g * l_f + a * h_cg) - l_f * c_sf * (g * l_r - a * h_cg)) * beta
                - (l_f.powi(2) * c_sf * (g * l_r - a * h_cg)
                    + l_r.powi(2) * c_sr * (g * l_f + a * h_cg))
                    * phidot
                    / v);
        let dbeta = mu / (v * l_wb)
            * (c_sf * (g * l_r - a * h_cg) * delta
                - (c_sr * (g * l_f + a * h_cg) + c_sf * (g * l_r - a * h_cg)) * beta
                + (c_sr * (g * l_f + a * h_cg) * l_r - c_sf * (g * l_r - a * h_cg) * l_f) * phidot
                    / v)
            - phidot;

        CarState {
            x: dx,
            y: dy,
            delta: *ddelta,
            v: *dv,
            phi: *dphi,
            phidot: dphidot,
            beta: dbeta,
        }
    }
}

impl Car {
    pub fn set_control(&mut self, control: CarControl) {
        self.control = control;
    }

    pub fn update(&mut self, dt: f64) {
        let k1 = compute_derivatives(&self.state, &self.params, &self.control);
        let k2 = compute_derivatives(&(self.state + k1 * (dt / 2.0)), &self.params, &self.control);
        let k3 = compute_derivatives(&(self.state + k2 * (dt / 2.0)), &self.params, &self.control);
        let k4 = compute_derivatives(&(self.state + k3 * dt), &self.params, &self.control);

        self.state += (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
        self.time += dt;
    }

    pub fn polygon(&self) -> Polygon {
        let x = self.state.x;
        let y = self.state.y;
        let phi = self.state.phi;
        let l = self.params.l;
        let w = self.params.w;

        Polygon::new(vec![
            [
                x + l / 2.0 * phi.cos() - w / 2.0 * phi.sin(),
                y + l / 2.0 * phi.sin() + w / 2.0 * phi.cos(),
            ],
            [
                x + l / 2.0 * phi.cos() + w / 2.0 * phi.sin(),
                y + l / 2.0 * phi.sin() - w / 2.0 * phi.cos(),
            ],
            [
                x - l / 2.0 * phi.cos() + w / 2.0 * phi.sin(),
                y - l / 2.0 * phi.sin() - w / 2.0 * phi.cos(),
            ],
            [
                x - l / 2.0 * phi.cos() - w / 2.0 * phi.sin(),
                y - l / 2.0 * phi.sin() + w / 2.0 * phi.cos(),
            ],
        ])
    }
}

mod car;

use std::{
    collections::VecDeque,
    time::Instant,
};

use car::{CarControl, Car};
use eframe::{
    egui::{
        self,
        plot::{Line, Plot, PlotPoint, Text},
        RichText,
    },
    emath::Align2,
};



fn main() {
    eframe::run_native(
        "Car simulator",
        eframe::NativeOptions::default(),
        Box::new(|_cc| {
            Box::new(MyApp {
                ..Default::default()
            })
        }),
    );
}

#[derive(Default)]
struct MyApp {
    car: Car,
    points: Vec<[f64; 2]>,
    state: SimState,
    elapsed_time: VecDeque<Instant>,
}

struct SimState {
    step_size: u32,
    control: CarControl,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            step_size: 10000,
            control: CarControl {
                a: 0.0,
                ddelta: 0.0,
            },
        }
    }
}


impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.elapsed_time.len() >= 100 {
            self.elapsed_time.pop_front();
        }
        self.elapsed_time.push_back(Instant::now());

        egui::Window::new("Settings").show(ctx, |ui| {
            ui.add(
                egui::Slider::new(&mut self.state.step_size, 1..=100000)
                    .text("Steps")
                    .logarithmic(true),
            );
        });

        if ctx.input().key_down(egui::Key::ArrowLeft) {
            self.state.control.ddelta = 0.04;
        }

        if ctx.input().key_down(egui::Key::ArrowRight) {
            self.state.control.ddelta = -0.04;
        }

        if ctx.input().key_down(egui::Key::ArrowUp) {
            self.state.control.a = 1.;
        }

        if ctx.input().key_down(egui::Key::ArrowDown) {
            self.state.control.a = -1.;
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            Plot::new("CarSim")
                .show_axes([true, true])
                .data_aspect(1.0)
                .show(ui, |plot| {
                    for _ in 0..self.state.step_size {
                        self.car.set_control(self.state.control);
                        self.car.update(0.000001);
                    }
                    self.points.push([self.car.state.x, self.car.state.y]);
                    plot.line(Line::new(self.points.clone()));
                    plot.polygon(self.car.polygon());
                    let [x, y] = plot.plot_bounds().min();
                    let w = plot.plot_bounds().width();
                    let h = plot.plot_bounds().height();
                    plot.text(
                        Text::new(
                            PlotPoint::new(x + w / 10., y + h / 10.),
                            RichText::new(format!("Time: {:.4}", self.car.time)).size(30.),
                        )
                        .anchor(Align2::LEFT_BOTTOM),
                    );
                });
        });

        ctx.request_repaint();

        let _fps =
            1.0 / self.elapsed_time[0].elapsed().as_secs_f64() * self.elapsed_time.len() as f64;
    }
}

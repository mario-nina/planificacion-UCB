#pragma once

#include <array>

#include "imt342_irb120_rviz/panel_values.hpp"

namespace imt342_irb120_rviz
{
struct TaskPreset
{
  const char * name;
  const char * description;
  int cycles;
  bool comparison;
  std::array<Joints, 4> waypoints_rad;
};

// Synthetic model configurations in J1–J6 order, with a nonzero virtual HOME.
inline constexpr Joints demo_home{0.05, -0.30, 0.51, 0.02, -0.37, 0.16};
inline constexpr std::array<TaskPreset, 5> task_presets{{
  {
    "Ciclo básico A–B–C–HOME",
    "Un recorrido de cuatro destinos con pausas articulares en cada punto.",
    1, false,
    {{{0.22, -0.34, 0.69, 0.16, -0.52, 0.32},
      {0.38, -0.25, 0.56, 0.24, -0.64, 0.28},
      {0.15, -0.18, 0.43, 0.11, -0.44, 0.24}, demo_home}}
  },
  {
    "Trabajo repetitivo",
    "Tres ciclos con retorno a la misma HOME virtual entre vueltas.",
    3, false,
    {{{0.37, -0.38, 0.73, 0.09, -0.62, 0.16},
      {0.58, -0.27, 0.51, 0.34, -0.47, 0.11},
      {0.32, -0.20, 0.43, 0.24, -0.50, 0.04}, demo_home}}
  },
  {
    "Inspección de tres posiciones",
    "Tres posturas con cambios de orientación de la muñeca y retorno al origen virtual.",
    1, false,
    {{{-0.21, -0.41, 0.66, -0.27, -0.59, 0.31},
      {-0.10, -0.30, 0.52, 0.03, -0.49, 0.51},
      {0.12, -0.24, 0.44, 0.43, -0.32, 0.68}, demo_home}}
  },
  {
    "Transferencia entre estaciones",
    "Recorrido entre estaciones virtuales A, B y C con retorno a HOME.",
    1, false,
    {{{-0.68, -0.38, 0.76, -0.35, -0.65, -0.22},
      {0.06, -0.58, 0.81, 0.17, -0.72, 0.02},
      {0.75, -0.35, 0.49, 0.39, -0.62, 0.43}, demo_home}}
  },
  {
    "Comparación de perfiles",
    "Dos ciclos con cálculo de duración y picos LSPB para dos escalas de velocidad.",
    2, true,
    {{{-0.95, -0.60, 0.79, -0.32, -0.70, -0.42},
      {0.92, -0.49, 0.33, 0.34, -0.43, 0.57},
      {0.42, -0.30, 0.70, 0.11, -0.55, 0.26}, demo_home}}
  },
}};
}  // namespace imt342_irb120_rviz

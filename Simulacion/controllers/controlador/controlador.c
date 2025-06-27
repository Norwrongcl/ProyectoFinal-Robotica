#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define INTERVALO 64
#define MATRIZ 8
#define TAM_CELDA 0.5
#define MAX_CAMINO 100
#define MAX_NODOS 1000
#define VELOCIDAD 4.0

typedef struct {
  int fila, col;
} Coordenada;

typedef struct {
  int fila, col;
  int g, h, f;
  int padre;
} Nodo;

int calcular_heuristica(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int buscar_ruta(int mapa[MATRIZ][MATRIZ], Coordenada inicio, Coordenada destino, Coordenada ruta[], int tam_max) {
  Nodo abiertos[MAX_NODOS];
  Nodo cerrados[MATRIZ * MATRIZ];
  int cant_abiertos = 0, cant_cerrados = 0;

  Nodo n_inicio = {inicio.fila, inicio.col, 0, calcular_heuristica(inicio.fila, inicio.col, destino.fila, destino.col), 0, -1};
  n_inicio.f = n_inicio.g + n_inicio.h;
  abiertos[cant_abiertos++] = n_inicio;

  while (cant_abiertos > 0) {
    int mejor = 0;
    for (int i = 1; i < cant_abiertos; i++) {
      if (abiertos[i].f < abiertos[mejor].f) mejor = i;
    }

    Nodo actual = abiertos[mejor];
    for (int i = mejor; i < cant_abiertos - 1; i++) abiertos[i] = abiertos[i + 1];
    cant_abiertos--;
    cerrados[cant_cerrados++] = actual;

    if (actual.fila == destino.fila && actual.col == destino.col) {
      int largo = 0;
      Nodo aux = actual;
      while (aux.padre != -1 && largo < tam_max) {
        ruta[largo++] = (Coordenada){aux.fila, aux.col};
        aux = cerrados[aux.padre];
      }
      ruta[largo++] = (Coordenada){inicio.fila, inicio.col};
      for (int i = 0; i < largo / 2; i++) {
        Coordenada tmp = ruta[i];
        ruta[i] = ruta[largo - i - 1];
        ruta[largo - i - 1] = tmp;
      }
      return largo;
    }

    int mov_fila[4] = {0, 1, 0, -1}, mov_col[4] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nueva_fila = actual.fila + mov_fila[d];
      int nueva_col = actual.col + mov_col[d];
      if (nueva_fila < 0 || nueva_col < 0 || nueva_fila >= MATRIZ || nueva_col >= MATRIZ) continue;
      if (mapa[nueva_fila][nueva_col] == 1) continue;

      int en_cerrados = 0;
      for (int i = 0; i < cant_cerrados; i++) {
        if (cerrados[i].fila == nueva_fila && cerrados[i].col == nueva_col) {
          en_cerrados = 1;
          break;
        }
      }
      if (en_cerrados) continue;

      int g = actual.g + 1;
      int h = calcular_heuristica(nueva_fila, nueva_col, destino.fila, destino.col);
      int f = g + h;

      int en_abiertos = -1;
      for (int i = 0; i < cant_abiertos; i++) {
        if (abiertos[i].fila == nueva_fila && abiertos[i].col == nueva_col) {
          en_abiertos = i;
          break;
        }
      }

      if (en_abiertos != -1) {
        if (f < abiertos[en_abiertos].f) {
          abiertos[en_abiertos].g = g;
          abiertos[en_abiertos].h = h;
          abiertos[en_abiertos].f = f;
          abiertos[en_abiertos].padre = cant_cerrados - 1;
        }
      } else if (cant_abiertos < MAX_NODOS) {
        Nodo vecino = {nueva_fila, nueva_col, g, h, f, cant_cerrados - 1};
        abiertos[cant_abiertos++] = vecino;
      }
    }
  }
  return 0;
}

int main() {
  wb_robot_init();

  double vel_izq = 0.0, vel_der = 0.0;
  int i;

  WbDeviceTag ruedas[4];
  char nombre_ruedas[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    ruedas[i] = wb_robot_get_device(nombre_ruedas[i]);
    wb_motor_set_position(ruedas[i], INFINITY);
  }

  WbDeviceTag sensores[2];
  char nombre_sensores[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    sensores[i] = wb_robot_get_device(nombre_sensores[i]);
    wb_distance_sensor_enable(sensores[i], INTERVALO);
  }

  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable(lidar, INTERVALO);
  wb_lidar_enable_point_cloud(lidar);

  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, INTERVALO);

  int mapa[MATRIZ][MATRIZ] = {0};
  Coordenada ruta[MAX_CAMINO];
  int largo_ruta = 0;
  int indice_actual = 0;
  bool destino_alcanzado = false;

  Coordenada origen = {MATRIZ / 2, MATRIZ / 2};
  Coordenada destino = {7, 7};

  largo_ruta = buscar_ruta(mapa, origen, destino, ruta, MAX_CAMINO);

  while (wb_robot_step(INTERVALO) != -1) {
    bool cerca_sensor = false;
    bool cerca_lidar = false;

    double distancias[2];
    for (i = 0; i < 2; i++) {
      distancias[i] = wb_distance_sensor_get_value(sensores[i]);
      if (distancias[i] < 950.0)
        cerca_sensor = true;
    }

    const double *pos = wb_gps_get_values(gps);
    float pos_x = pos[0];
    float pos_y = pos[2];

    const float *barrido = wb_lidar_get_range_image(lidar);
    int res = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);

    for (int j = 0; j < res; j++) {
      double angulo = -fov / 2 + j * (fov / res);
      double distancia = barrido[j];
      if (isinf(distancia)) continue;

      if (distancia < 1.0) {
        float obs_x = pos_x + distancia * cos(angulo);
        float obs_y = pos_y + distancia * sin(angulo);
        int celda_x = (int)((obs_x + MATRIZ * TAM_CELDA / 2) / TAM_CELDA);
        int celda_y = (int)((obs_y + MATRIZ * TAM_CELDA / 2) / TAM_CELDA);
        if (celda_x >= 0 && celda_x < MATRIZ && celda_y >= 0 && celda_y < MATRIZ)
          mapa[celda_x][celda_y] = 1;
      }

      if (distancia < 0.3)
        cerca_lidar = true;
    }

    if (!destino_alcanzado && largo_ruta > 0) {
      int celda_actual_x = (int)((pos_x + MATRIZ * TAM_CELDA / 2) / TAM_CELDA);
      int celda_actual_y = (int)((pos_y + MATRIZ * TAM_CELDA / 2) / TAM_CELDA);

      Coordenada punto_actual = ruta[indice_actual];
      if (celda_actual_x == punto_actual.fila && celda_actual_y == punto_actual.col) {
        indice_actual++;
        if (indice_actual >= largo_ruta) {
          destino_alcanzado = true;
        }
      }
    }

    if (destino_alcanzado) {
      vel_izq = 0.0;
      vel_der = 0.0;
    } else if (cerca_lidar || cerca_sensor) {
      vel_izq = 1.0;
      vel_der = -1.0;
    } else {
      vel_izq = VELOCIDAD;
      vel_der = VELOCIDAD;
    }

    for (i = 0; i < 4; i++) {
      if (i % 2 == 0)
        wb_motor_set_velocity(ruedas[i], vel_izq);
      else
        wb_motor_set_velocity(ruedas[i], vel_der);
    }

    fflush(stdout);
  }

  wb_robot_cleanup();
  return 0;
}


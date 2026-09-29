# Software-de-control-de-la-Smart-Ball
// ================================================================
// SMART BALL  (Adafruit MPU6050) final
// ========================================================= =======

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "esp_sleep.h"

Adafruit_MPU6050 mpu;

// ---------------- PINES ----------------

const int buzzer            = 18;
const int BATTERY_ADC_PIN   = 33;
const int pin_SCL           = 22;
const int pin_SDA           = 21;
const int pin_pulsador      = 4;
const int pin_potenciometro = 32;

// ---------------- PARÁMETROS DE FUNCIONAMIENTO ----------------

// Factor experimental de calibración de la medida de batería.
// Se ajusta comparando la tensión calculada por el ESP32
// con la tensión real medida mediante multímetro.
const float BATTERY_CALIBRATION_FACTOR = 4.25;

// Umbrales de detección de movimiento del MPU6050
const float UMBRAL_DESPERTAR  = 2.0;
const float UMBRAL_MOVIMIENTO = 1.5;

// Tiempo sin movimiento necesario para entrar en modo QUIETA
const unsigned long TIEMPO_REPOSO = 1000;   // 1 segundo

// Tiempo en modo QUIETA antes de entrar en Deep Sleep
const unsigned long TIEMPO_SLEEP = 60000;   // 60 segundos

// Límites de volumen
const int VOLUMEN_MAX_REPOSO      = 255;
const int VOLUMEN_BASE_MOVIMIENTO = 128;


// ---------------- CONTROL DE VOLUMEN ----------------

// Filtro utilizado para suavizar los cambios de volumen
float alpha  = 0.35;
float k_sens = 10.0;
float expo   = 1.8;

int volumen_cascabel_filtrado = VOLUMEN_BASE_MOVIMIENTO;


// ---------------- VARIABLES DE ESTADO ----------------

// Control de movimiento
unsigned long ultimo_movimiento = 0;

bool Estado_reposo = false;
bool moviendose = false;

// Control del tiempo en modo QUIETA
unsigned long inicioModoQuieta = 0;
bool contandoSleep = false;
bool sleepAlcanzado = false;

// Lecturas anteriores del acelerómetro
float inicialX = 0;
float inicialY = 0;
float inicialZ = 0;

bool Lectura_inicial = true;

// Estado general del sistema
bool interruptorAnterior = false;

// Volumen
int volumen_maestro = 120;
int volumen_cascabel = VOLUMEN_BASE_MOVIMIENTO;

// ---------------- SONIDOS ----------------

// Secuencia de sonido durante el movimiento
const int notas_cascabel[] = {
    330, 2600, 330, 2600, 330, 2600
};

const int duraciones_cascabel[] = {
    30, 35, 30, 35, 30, 35
};

const int num_notas_cascabel = 6;


// Secuencia de sonido en modo QUIETA
const int notas_quieta[] = {
    330, 2600, 330, 2600, 330
};

const int duraciones_quieta[] = {
    180, 180, 180, 180, 180
};

const int pausas_quieta[] = {
    200, 200, 250, 200, 800
};

const int num_notas_quieta = 5;


// Estados posibles del sistema de sonido
enum TipoSonido
{
    NINGUNO,
    CASCABEL,
    QUIETA
};

TipoSonido sonido_actual = NINGUNO;

// Variables de control de reproducción
int notaActual = 0;
unsigned long tiempoNota = 0;
bool enPausa = false;

// =====================================================
// FUNCIONES
// =====================================================

// Genera un tono aplicando el volumen maestro del potenciómetro
void tonoConVolumen(int frecuencia, int limiteMax = 255)
{
    int volumenFinal = (limiteMax * volumen_maestro) / 255;

    if (volumenFinal > 0)
    {
        ledcWriteTone(buzzer, frecuencia);
        ledcWrite(buzzer, volumenFinal);
    }
    else
    {
        ledcWrite(buzzer, 0);
    }
}


// Genera un tono con un nivel de volumen fijo
void tonoFijo(int frecuencia, int volumen = 200)
{
    ledcWriteTone(buzzer, frecuencia);
    ledcWrite(buzzer, volumen);
}

// Indica mediante pitidos el nivel aproximado de batería
void pitidosBateria()
{
    long sumaLecturasADC = 0;

    // Se realizan 100 lecturas para obtener un valor medio
    for (int i = 0; i < 100; i++)
    {
        sumaLecturasADC += analogRead(BATTERY_ADC_PIN);
    }

    // Conversión de la lectura media del ADC a tensión de batería
    float tensionBateria =
        ((sumaLecturasADC / 100.0) * 3.3 / 4095.0) *
        BATTERY_CALIBRATION_FACTOR;

    if (tensionBateria >= 7.9)
    {
        // Nivel alto: 3 pitidos
        for (int i = 0; i < 3; i++)
        {
            tonoFijo(1200);
            delay(200);
            ledcWrite(buzzer, 0);
            delay(200);
        }
    }
    else if (tensionBateria >= 7.5)
    {
        // Nivel medio: 2 pitidos
        for (int i = 0; i < 2; i++)
        {
            tonoFijo(1000);
            delay(200);
            ledcWrite(buzzer, 0);
            delay(200);
        }
    }
    else if (tensionBateria >= 7.4)
    {
        // Nivel bajo: 1 pitido
        tonoFijo(800);
        delay(200);
        ledcWrite(buzzer, 0);
        delay(200);
    }
    else
    {
        // Nivel bateria baja: 5 pitidos
        for (int i = 0; i < 5; i++)
        {
            tonoFijo(400);
            delay(300);
            ledcWrite(buzzer, 0);
            delay(300);
        }
    }
}

void setup()
{
    // Comunicación serie
    Serial.begin(115200);

    // Inicialización del bus I2C del MPU6050
    Wire.begin(pin_SDA, pin_SCL);

    // Configuración del buzzer mediante PWM
    ledcAttach(buzzer, 2000, 8);
    ledcWrite(buzzer, 0);

    // Configuración del interruptor físico
    pinMode(pin_pulsador, INPUT_PULLUP);

    // Resolución del ADC del ESP32: 12 bits (0-4095)
    analogReadResolution(12);

    delay(600);

    // Inicialización del MPU6050
    if (!mpu.begin())
    {
        while (1)
        {
            delay(100);
        }
    }

    // Configuración del MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G); // Rango del acelerómetro: ±4 g
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // Indicación inicial del nivel de batería
    pitidosBateria();
}

void loop()
{
    // Lectura del potenciómetro y ajuste no lineal del volumen maestro
    float p = analogRead(pin_potenciometro) / 4095.0;
    volumen_maestro = (int)(255.0 * pow(p, 2.3));

    // Lectura del interruptor físico
    bool interruptorON = (digitalRead(pin_pulsador) == LOW);

    // Activación del sistema al cambiar el interruptor de OFF a ON
    if (interruptorON && !interruptorAnterior)
    {

        // Reinicio de las variables de movimiento
        Lectura_inicial = true;
        ultimo_movimiento = millis();
        Estado_reposo = false;
        moviendose = false;

        // Reinicio del volumen filtrado
        volumen_cascabel_filtrado = VOLUMEN_BASE_MOVIMIENTO;

        // Reinicio del temporizador de Deep Sleep
        contandoSleep = false;
        inicioModoQuieta = 0;
        sleepAlcanzado = false;
    }

    // Apagado mediante el interruptor físico
    if (!interruptorON)
    {
        pararSonido();

        interruptorAnterior = false;

        delay(50);
        return;
    }

    // Guarda el estado actual del interruptor
    interruptorAnterior = interruptorON;


    // ---------------- LECTURA DEL MPU6050 ----------------

    // Variables requeridas por la librería para almacenar
    // aceleración, velocidad angular y temperatura.
    // En este programa se utilizan los datos del acelerómetro.
    sensors_event_t a, g, temp;

    // Obtención de una nueva lectura del MPU6050
    mpu.getEvent(&a, &g, &temp);

    // Aceleración medida en los tres ejes
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;


    // ---------------- DETECCIÓN DE MOVIMIENTO ----------------

    // Primera lectura utilizada como referencia
    if (Lectura_inicial)
    {
        inicialX = ax;
        inicialY = ay;
        inicialZ = az;

        Lectura_inicial = false;
        return;
    }

    // Variación total de aceleración respecto a la lectura anterior
    float Var_total =
        fabs(ax - inicialX) +
        fabs(ay - inicialY) +
        fabs(az - inicialZ);

    // Selección del umbral según el estado de movimiento
    float umbral_actual =
        moviendose ? UMBRAL_MOVIMIENTO : UMBRAL_DESPERTAR;


    // ---------------- PELOTA EN MOVIMIENTO ----------------

    if (Var_total > umbral_actual)
    {
        ultimo_movimiento = millis();

        Estado_reposo = false;
        moviendose = true;

        // Cancela el temporizador de Deep Sleep
        contandoSleep = false;
        inicioModoQuieta = 0;
        sleepAlcanzado = false;

        // Movimiento detectado por encima del umbral
        float exceso = Var_total - UMBRAL_MOVIMIENTO;

        if (exceso < 0)
        {
            exceso = 0;
        }

        // Cálculo del volumen en función del movimiento
        int volumen_calculado =
            VOLUMEN_BASE_MOVIMIENTO +
            (int)(k_sens * pow(exceso, expo));

        // Limita el volumen al rango permitido
        volumen_calculado =
            constrain(
                volumen_calculado,
                VOLUMEN_BASE_MOVIMIENTO,
                255
            );

        // Suavizado de los cambios de volumen
        volumen_cascabel =
            volumen_cascabel_filtrado * (1.0 - alpha) +
            volumen_calculado * alpha;

        volumen_cascabel_filtrado = volumen_cascabel;

        // Inicia el sonido asociado al movimiento
        if (sonido_actual != CASCABEL)
        {
            iniciarCascabel();
        }
    }


    // ---------------- PELOTA SIN MOVIMIENTO ----------------

    else
    {
        // Reducción progresiva del volumen hacia el nivel base
        volumen_cascabel =
            volumen_cascabel_filtrado * (1.0 - alpha) +
            VOLUMEN_BASE_MOVIMIENTO * alpha;

        volumen_cascabel_filtrado = volumen_cascabel;

        // Comprueba si se ha alcanzado el tiempo de reposo
        if (millis() - ultimo_movimiento >= TIEMPO_REPOSO)
        {
            Estado_reposo = true;
            moviendose = false;

            // Inicio del temporizador al entrar en modo QUIETA
            if (!contandoSleep)
            {
                inicioModoQuieta = millis();
                contandoSleep = true;
            }

            // Reproducción del sonido de modo QUIETA
            if (!sleepAlcanzado &&
                sonido_actual != QUIETA)
            {
                iniciarMelodiaQuieta();
            }

            // Entrada en Deep Sleep tras permanecer en modo QUIETA
            if (millis() - inicioModoQuieta >= TIEMPO_SLEEP &&
                !sleepAlcanzado)
            {
                sleepAlcanzado = true;

                // Detiene el sonido del modo QUIETA
                pararSonido();

                // Aviso acústico previo al Deep Sleep
                tonoFijo(800, 180);
                delay(400);
                ledcWrite(buzzer, 0);

                delay(1000);

                tonoFijo(800, 180);
                delay(1000);
                ledcWrite(buzzer, 0);

                // Silencio antes de entrar en Deep Sleep
                pararSonido();

                delay(500);

                esp_deep_sleep_start();
            }
        }
        else
        {
            // Mantiene el sonido de movimiento mientras
            // no se haya alcanzado el tiempo de reposo
            if (sonido_actual != CASCABEL)
            {
                iniciarCascabel();
            }
        }
    }

    // Guarda la lectura actual para la siguiente comparación
    inicialX = ax;
    inicialY = ay;
    inicialZ = az;

    // Actualización de la secuencia de sonido
    actualizarSonido();

    delay(15);
}

// ===================================================
// GESTIÓN DE SONIDOS
// ===================================================

// Inicia la secuencia de sonido asociada al movimiento
void iniciarCascabel()
{
    sonido_actual = CASCABEL;

    notaActual = 0;
    tiempoNota = millis();
    enPausa = false;

    tonoConVolumen(
        notas_cascabel[0],
        volumen_cascabel
    );
}


// Inicia la secuencia de sonido del modo QUIETA
void iniciarMelodiaQuieta()
{
    sonido_actual = QUIETA;

    notaActual = 0;
    tiempoNota = millis();
    enPausa = false;

    tonoConVolumen(
        notas_quieta[0],
        VOLUMEN_MAX_REPOSO
    );
}


// Detiene cualquier sonido activo
void pararSonido()
{
    sonido_actual = NINGUNO;
    ledcWrite(buzzer, 0);
}


// Actualiza la reproducción de las secuencias de sonido
void actualizarSonido()
{
    if (sonido_actual == NINGUNO)
    {
        return;
    }

    unsigned long ahora = millis();


    // ---------------- SONIDO DE MOVIMIENTO ----------------

    if (sonido_actual == CASCABEL)
    {
        if (!enPausa)
        {
            // Comprueba si ha terminado la nota actual
            if (ahora - tiempoNota >= duraciones_cascabel[notaActual])
            {
                enPausa = true;
                tiempoNota = ahora;
                ledcWrite(buzzer, 0);
            }
        }
        else
        {
            // Pequeña pausa entre notas
            if (ahora - tiempoNota >= 5)
            {
                notaActual++;

                // Reinicia la secuencia al llegar a la última nota
                if (notaActual >= num_notas_cascabel)
                {
                    notaActual = 0;
                }

                tonoConVolumen(
                    notas_cascabel[notaActual],
                    volumen_cascabel
                );

                tiempoNota = ahora;
                enPausa = false;
            }
        }
    }


    // ---------------- SONIDO MODO QUIETA ----------------

    else if (sonido_actual == QUIETA)
    {
        if (!enPausa)
        {
            // Comprueba si ha terminado la nota actual
            if (ahora - tiempoNota >= duraciones_quieta[notaActual])
            {
                enPausa = true;
                tiempoNota = ahora;
                ledcWrite(buzzer, 0);
            }
        }
        else
        {
            // Comprueba si ha terminado la pausa asociada a la nota
            if (ahora - tiempoNota >= pausas_quieta[notaActual])
            {
                notaActual++;

                // Fin de la secuencia
                if (notaActual >= num_notas_quieta)
                {
                    if (Estado_reposo)
                    {
                        // Repite la secuencia mientras la pelota siga quieta
                        notaActual = 0;
                        tiempoNota = ahora;
                        enPausa = false;

                        tonoConVolumen(
                            notas_quieta[0],
                            VOLUMEN_MAX_REPOSO
                        );
                    }
                    else
                    {
                        pararSonido();
                    }

                    return;
                }

                tonoConVolumen(
                    notas_quieta[notaActual],
                    VOLUMEN_MAX_REPOSO
                );

                tiempoNota = ahora;
                enPausa = false;
            }
        }
    }
}

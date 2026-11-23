#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "driver/pcnt.h"

// =========================================================================
// 1. DEFINICIÓN DE PINES: L298N MINI + ENCODERS POLOLU + LEDS TESTIGO
// =========================================================================

// Motor 1: Elevación del Brazo (Subir - Bajar)
#define M1_IN1_PIN				26	// L298N Mini 1 IN1
#define M1_IN2_PIN				27	// L298N Mini 1 IN2
#define ENC1_C1_PIN				34	// Pololu Motor 1 C1 (Canal A)
#define ENC1_C2_PIN				35	// Pololu Motor 1 C2 (Canal B)
#define PCNT_M1_UNIT			PCNT_UNIT_0

// Motor 2: Pinza (Apretar - Soltar)
#define M2_IN3_PIN				12	// L298N Mini 1 IN3
#define M2_IN4_PIN				13	// L298N Mini 1 IN4
#define ENC2_C1_PIN				32	// Pololu Motor 2 C1 (Canal A)
#define ENC2_C2_PIN				33	// Pololu Motor 2 C2 (Canal B)
#define PCNT_M2_UNIT			PCNT_UNIT_1

// Motor 3: Base Giratoria / Yaw (Segundo L298N Mini)
#define M3_IN1_PIN				25	// L298N Mini 2 IN1
#define M3_IN2_PIN				14	// L298N Mini 2 IN2
#define ENC3_C1_PIN				18	// Pololu Motor 3 C1 (Canal A)
#define ENC3_C2_PIN				19	// Pololu Motor 3 C2 (Canal B)
#define PCNT_M3_UNIT			PCNT_UNIT_2

// Configuración PWM (1000 Hz para el chip MX1508 del L298N Mini, 10 bits: 0 a 1023)
#define PWM_FREQ				1000
#define PWM_RES					10
#define PWM_CH_M1_IN1			0
#define PWM_CH_M1_IN2			1
#define PWM_CH_M2_IN3			2
#define PWM_CH_M2_IN4			3
#define PWM_CH_M3_IN1			4
#define PWM_CH_M3_IN2			5

// LEDs indicadores
#define PIN_LED_PINZA			4
#define PIN_LED_INTEGRADO		2
const int NUM_LEDS_VIRTUALES = 8;

// =========================================================================
// 2. ESCALAS MECÁNICAS Y BANDA MUERTA DE REPOSO (3-DOF)
// =========================================================================

#define PULSOS_POR_GRADO_BRAZO	2.333f
#define PULSOS_CIERRE_PINZA		210.0f
#define PULSOS_POR_GRADO_YAW	2.333f

#define ANGULO_MIN_BRAZO		-40.0f
#define ANGULO_MAX_BRAZO		40.0f
#define ANGULO_MIN_YAW			-60.0f
#define ANGULO_MAX_YAW			60.0f

// En 0.0f para que el motor se detenga totalmente en reposo
#define PWM_FEEDFORWARD_GRAV	0.0f
#define MAX_PASO_RAMPA_BRAZO	2.5f
#define MAX_PASO_RAMPA_PINZA	6.0f
#define MAX_PASO_RAMPA_YAW		2.0f

// Velocidad de avance cinemático por ciclo de 10ms (65 pulsos/s = ~28 grados/s)
#define PASO_AVANCE_M1			0.65f
#define PASO_AVANCE_M3			0.65f

// Histéresis angular y zona cero para mantener posición fija e independiente
#define ZONA_CERO_PITCH			4.5f
#define HIST_ESTABILIDAD_PITCH	3.5f
#define ZONA_CERO_YAW			4.5f
#define HIST_ESTABILIDAD_YAW	3.5f

// Tolerancia en pulsos dentro de la cual el motor se apaga por completo
#define TOLERANCIA_PULSOS_M1	4.0f
#define TOLERANCIA_PULSOS_M2	5.0f
#define TOLERANCIA_PULSOS_M3	4.0f

// Signo de los motores/encoders (cambia a -1 con comando FLIP_M1 / FLIP_M2 / FLIP_M3)
volatile int8_t dir_encoder_m1 = 1;
volatile int8_t dir_encoder_m2 = 1;
volatile int8_t dir_encoder_m3 = 1;

// Umbrales dinámicos de contracción muscular (sEMG) con histéresis
volatile float umbral_apretar_pinza = 24.0f;
volatile float umbral_soltar_pinza = 18.0f;

// =========================================================================
// 3. UUIDs OFICIALES DEL PROTOCOLO BLE GATT DEL MYO ARMBAND
// =========================================================================

static BLEUUID MYO_SVC_CONTROL("d5060001-a904-deb9-4748-2c7f4a124842");
static BLEUUID MYO_CHAR_COMMAND("d5060401-a904-deb9-4748-2c7f4a124842");

static BLEUUID MYO_SVC_IMU("d5060002-a904-deb9-4748-2c7f4a124842");
static BLEUUID MYO_CHAR_IMU_DATA("d5060402-a904-deb9-4748-2c7f4a124842");

static BLEUUID MYO_SVC_EMG("d5060005-a904-deb9-4748-2c7f4a124842");
static const char* MYO_CHARS_EMG[4] = {
	"d5060105-a904-deb9-4748-2c7f4a124842",
	"d5060205-a904-deb9-4748-2c7f4a124842",
	"d5060305-a904-deb9-4748-2c7f4a124842",
	"d5060405-a904-deb9-4748-2c7f4a124842"
};

volatile float angulo_pitch_actual = 0.0f;
volatile float angulo_yaw_actual = 0.0f;
volatile float envolvente_emg = 0.0f;
volatile float setpoint_brazo_pulsos = 0.0f;
volatile float setpoint_pinza_pulsos = 0.0f;
volatile float setpoint_yaw_pulsos = 0.0f;
volatile float pos_estimada_m1 = 0.0f;
volatile float pos_estimada_m3 = 0.0f;
volatile int32_t posicion_acumulada_m1 = 0;
volatile int32_t posicion_acumulada_m2 = 0;
volatile int32_t posicion_acumulada_m3 = 0;
volatile int leds_encendidos_brazo = 4;
volatile bool estado_pinza_cerrada = false;
volatile bool myo_conectado = false;
volatile bool myo_encontrado = false;
volatile bool pitch_calibrado = false;
volatile bool solicitar_vibracion = false;
float pitch_offset_inicial = 0.0f;
float yaw_offset_inicial = 0.0f;

static BLEAdvertisedDevice* myo_detectado = nullptr;
static BLERemoteCharacteristic* pCharComandoGlobal = nullptr;

// =========================================================================
// 4. CONTROLADOR PD CON BANDA MUERTA Y ENCODERS DE 32 BITS SIN OVERFLOW
// =========================================================================

struct ControladorPID {
	float kp;
	float ki;
	float kd;
	float integral;
	float error_previo;
	float max_integral;
	float max_salida;

	void reiniciar() {
		integral = 0.0f;
		error_previo = 0.0f;
	}

	float calcular(float referencia, float medicion, float dt, float banda_muerta) {
		float error = referencia - medicion;

		// Si el motor ya llegó a la posición objetivo, apaga el motor y limpia la integral
		if (fabsf(error) <= banda_muerta) {
			integral = 0.0f;
			error_previo = error;
			return 0.0f;
		}

		integral += error * dt;
		integral = constrain(integral, -max_integral, max_integral);
		float derivada = (error - error_previo) / dt;
		error_previo = error;
		float u = (kp * error) + (ki * integral) + (kd * derivada);
		return constrain(u, -max_salida, max_salida);
	}
};

// Ki = 0.0f para evitar que el integrador deje los motores girando en reposo
ControladorPID pid_elevacion = {5.0f, 0.0f, 0.35f, 0.0f, 0.0f, 200.0f, 850.0f};
ControladorPID pid_pinza = {4.5f, 0.0f, 0.25f, 0.0f, 0.0f, 150.0f, 700.0f};
ControladorPID pid_base = {5.0f, 0.0f, 0.35f, 0.0f, 0.0f, 200.0f, 850.0f};

void inicializar_pcnt_x4(pcnt_unit_t unidad, int pin_c1, int pin_c2) {
	pinMode(pin_c1, INPUT_PULLUP);
	pinMode(pin_c2, INPUT_PULLUP);

	pcnt_config_t cfg_ch0 = {};
	cfg_ch0.pulse_gpio_num = pin_c1;
	cfg_ch0.ctrl_gpio_num = pin_c2;
	cfg_ch0.unit = unidad;
	cfg_ch0.channel = PCNT_CHANNEL_0;
	cfg_ch0.pos_mode = PCNT_COUNT_INC;
	cfg_ch0.neg_mode = PCNT_COUNT_DEC;
	cfg_ch0.lctrl_mode = PCNT_MODE_REVERSE;
	cfg_ch0.hctrl_mode = PCNT_MODE_KEEP;
	cfg_ch0.counter_h_lim = 30000;
	cfg_ch0.counter_l_lim = -30000;
	pcnt_unit_config(&cfg_ch0);

	pcnt_config_t cfg_ch1 = {};
	cfg_ch1.pulse_gpio_num = pin_c2;
	cfg_ch1.ctrl_gpio_num = pin_c1;
	cfg_ch1.unit = unidad;
	cfg_ch1.channel = PCNT_CHANNEL_1;
	cfg_ch1.pos_mode = PCNT_COUNT_DEC;
	cfg_ch1.neg_mode = PCNT_COUNT_INC;
	cfg_ch1.lctrl_mode = PCNT_MODE_REVERSE;
	cfg_ch1.hctrl_mode = PCNT_MODE_KEEP;
	cfg_ch1.counter_h_lim = 30000;
	cfg_ch1.counter_l_lim = -30000;
	pcnt_unit_config(&cfg_ch1);

	pcnt_set_filter_value(unidad, 100);
	pcnt_filter_enable(unidad);

	pcnt_counter_pause(unidad);
	pcnt_counter_clear(unidad);
	pcnt_counter_resume(unidad);
}

// Lee el delta y limpia el registro para acumular en 32 bits sin desbordamiento
int32_t actualizar_posicion_encoder(pcnt_unit_t unidad, volatile int32_t &acumulador, int8_t dir_signo, float u_actual = 0.0f) {
	int16_t delta = 0;
	pcnt_get_counter_value(unidad, &delta);
	if (delta != 0) {
		pcnt_counter_clear(unidad);
		int32_t paso = (int32_t)delta * (int32_t)dir_signo;
		if (u_actual > 25.0f && paso < 0) {
			paso = -paso;
		} else if (u_actual < -25.0f && paso > 0) {
			paso = -paso;
		}
		acumulador += paso;
	}
	return acumulador;
}

// =========================================================================
// 5. ETAPA DE POTENCIA (L298N MINI) Y TAREA DETERMINISTA PID (CORE 1)
// =========================================================================

void comandar_motor_mini(int ch_pwm_fwd, int ch_pwm_rev, float senal_control) {
	uint32_t magnitud = (uint32_t)fabsf(senal_control);

	// Corte total de energía cuando está dentro de la zona muerta
	if (magnitud < 35) {
		ledcWrite(ch_pwm_fwd, 0);
		ledcWrite(ch_pwm_rev, 0);
		return;
	}

	// Mapeo suave desde el umbral de arranque hasta el máximo de 10 bits (1023)
	uint32_t pwm_duty = map(constrain(magnitud, 35, 850), 35, 850, 220, 1023);

	if (senal_control > 0.0f) {
		ledcWrite(ch_pwm_rev, 0);
		ledcWrite(ch_pwm_fwd, pwm_duty);
	} else {
		ledcWrite(ch_pwm_fwd, 0);
		ledcWrite(ch_pwm_rev, pwm_duty);
	}
}

float aplicar_rampa(float valor_actual, float objetivo, float paso_maximo) {
	float diferencia = objetivo - valor_actual;
	if (diferencia > paso_maximo) {
		return valor_actual + paso_maximo;
	} else if (diferencia < -paso_maximo) {
		return valor_actual - paso_maximo;
	}
	return objetivo;
}

void tarea_lazo_cerrado_pid(void* pvParameters) {
	TickType_t ultimo_ciclo = xTaskGetTickCount();
	const TickType_t periodo_10ms = pdMS_TO_TICKS(10); // 100 Hz
	const float dt = 0.01f;

	float sp_suave_pinza = 0.0f;

	while (true) {
		// Limpiar registros PCNT para evitar desbordamiento
		int16_t delta_tmp = 0;
		pcnt_get_counter_value(PCNT_M1_UNIT, &delta_tmp);
		if (delta_tmp != 0) pcnt_counter_clear(PCNT_M1_UNIT);
		pcnt_get_counter_value(PCNT_M2_UNIT, &delta_tmp);
		if (delta_tmp != 0) pcnt_counter_clear(PCNT_M2_UNIT);
		pcnt_get_counter_value(PCNT_M3_UNIT, &delta_tmp);
		if (delta_tmp != 0) pcnt_counter_clear(PCNT_M3_UNIT);

		if (myo_conectado) {
			float u_brazo = 0.0f;
			float u_pinza = 0.0f;
			float u_yaw = 0.0f;

			// 1. MOTOR 1 (ELEVACIÓN SUBIR-BAJAR): Avanza hacia setpoint_brazo_pulsos y se detiene fijo
			float sp_m1 = setpoint_brazo_pulsos;
			float error_m1 = sp_m1 - pos_estimada_m1;
			if (fabsf(error_m1) <= 0.8f) {
				u_brazo = 0.0f;
				pos_estimada_m1 = sp_m1;
				posicion_acumulada_m1 = (int32_t)roundf(pos_estimada_m1);
				pid_elevacion.reiniciar();
			} else {
				float u_pd_m1 = pid_elevacion.calcular(sp_m1, pos_estimada_m1, dt, 0.8f);
				float mag_m1 = constrain(fabsf(u_pd_m1) + 460.0f, 520.0f, 820.0f);
				float signo_m1 = (error_m1 > 0.0f) ? 1.0f : -1.0f;
				u_brazo = signo_m1 * mag_m1 * (float)dir_encoder_m1;

				if (error_m1 > 0.0f) {
					pos_estimada_m1 += PASO_AVANCE_M1;
					if (pos_estimada_m1 > sp_m1) pos_estimada_m1 = sp_m1;
				} else {
					pos_estimada_m1 -= PASO_AVANCE_M1;
					if (pos_estimada_m1 < sp_m1) pos_estimada_m1 = sp_m1;
				}
				posicion_acumulada_m1 = (int32_t)roundf(pos_estimada_m1);
			}

			// 2. MOTOR 2 (PINZA CON CONTROL DE IMPEDANCIA): Apagado total con puño suelto, fuerza activa al apretar
			if (!estado_pinza_cerrada) {
				u_pinza = 0.0f;
				sp_suave_pinza = 0.0f;
				posicion_acumulada_m2 = 0;
				pid_pinza.reiniciar();
			} else {
				sp_suave_pinza = aplicar_rampa(sp_suave_pinza, setpoint_pinza_pulsos, MAX_PASO_RAMPA_PINZA);
				posicion_acumulada_m2 = (int32_t)roundf(sp_suave_pinza);
				float extra_fuerza = constrain((envolvente_emg - umbral_apretar_pinza) * 10.0f, 0.0f, 380.0f);
				float mag_pinza = constrain(470.0f + extra_fuerza, 470.0f, 850.0f);
				u_pinza = mag_pinza * (float)dir_encoder_m2;
			}

			// 3. MOTOR 3 (BASE GIRATORIA / YAW): Exactamente igual que M1 pero gobernado por setpoint_yaw_pulsos
			float sp_m3 = setpoint_yaw_pulsos;
			float error_m3 = sp_m3 - pos_estimada_m3;
			if (fabsf(error_m3) <= 0.8f) {
				u_yaw = 0.0f;
				pos_estimada_m3 = sp_m3;
				posicion_acumulada_m3 = (int32_t)roundf(pos_estimada_m3);
				pid_base.reiniciar();
			} else {
				float u_pd_m3 = pid_base.calcular(sp_m3, pos_estimada_m3, dt, 0.8f);
				float mag_m3 = constrain(fabsf(u_pd_m3) + 460.0f, 520.0f, 820.0f);
				float signo_m3 = (error_m3 > 0.0f) ? 1.0f : -1.0f;
				u_yaw = signo_m3 * mag_m3 * (float)dir_encoder_m3;

				if (error_m3 > 0.0f) {
					pos_estimada_m3 += PASO_AVANCE_M3;
					if (pos_estimada_m3 > sp_m3) pos_estimada_m3 = sp_m3;
				} else {
					pos_estimada_m3 -= PASO_AVANCE_M3;
					if (pos_estimada_m3 < sp_m3) pos_estimada_m3 = sp_m3;
				}
				posicion_acumulada_m3 = (int32_t)roundf(pos_estimada_m3);
			}

			comandar_motor_mini(PWM_CH_M1_IN1, PWM_CH_M1_IN2, u_brazo);
			comandar_motor_mini(PWM_CH_M2_IN3, PWM_CH_M2_IN4, u_pinza);
			comandar_motor_mini(PWM_CH_M3_IN1, PWM_CH_M3_IN2, u_yaw);
		} else {
			sp_suave_pinza = 0.0f;
			comandar_motor_mini(PWM_CH_M1_IN1, PWM_CH_M1_IN2, 0.0f);
			comandar_motor_mini(PWM_CH_M2_IN3, PWM_CH_M2_IN4, 0.0f);
			comandar_motor_mini(PWM_CH_M3_IN1, PWM_CH_M3_IN2, 0.0f);
		}

		vTaskDelayUntil(&ultimo_ciclo, periodo_10ms);
	}
}

// =========================================================================
// 6. CALLBACKS BLE: LECTURA DE ORIENTACIÓN (IMU) Y MÚSCULO (sEMG)
// =========================================================================

static void callback_imu(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
	if (length < 8) return;

	int16_t qw_i = (int16_t)((pData[1] << 8) | pData[0]);
	int16_t qx_i = (int16_t)((pData[3] << 8) | pData[2]);
	int16_t qy_i = (int16_t)((pData[5] << 8) | pData[4]);
	int16_t qz_i = (int16_t)((pData[7] << 8) | pData[6]);

	float qw = qw_i / 16384.0f;
	float qx = qx_i / 16384.0f;
	float qy = qy_i / 16384.0f;
	float qz = qz_i / 16384.0f;

	float sin_pitch = 2.0f * (qw * qy - qz * qx);
	sin_pitch = constrain(sin_pitch, -1.0f, 1.0f);
	float pitch_bruto = asinf(sin_pitch) * (180.0f / PI);

	float yaw_bruto = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / PI);

	static float pitch_filtrado = 0.0f;
	static float pitch_estable = 0.0f;
	static float yaw_filtrado = 0.0f;
	static float yaw_estable = 0.0f;

	if (!pitch_calibrado) {
		pitch_offset_inicial = pitch_bruto;
		yaw_offset_inicial = yaw_bruto;
		pitch_filtrado = 0.0f;
		pitch_estable = 0.0f;
		yaw_filtrado = 0.0f;
		yaw_estable = 0.0f;
		pitch_calibrado = true;
	}

	// 1. Filtrado y acotamiento de Pitch (Elevación M1)
	float pitch_relativo = pitch_bruto - pitch_offset_inicial;
	pitch_filtrado = (0.25f * pitch_relativo) + (0.75f * pitch_filtrado);
	float pitch_acotado = constrain(pitch_filtrado, ANGULO_MIN_BRAZO, ANGULO_MAX_BRAZO);
	if (fabsf(pitch_acotado) <= ZONA_CERO_PITCH) {
		pitch_acotado = 0.0f;
	}

	// 2. Filtrado y acotamiento de Yaw (Base Giratoria M3)
	float yaw_relativo = yaw_bruto - yaw_offset_inicial;
	if (yaw_relativo > 180.0f) yaw_relativo -= 360.0f;
	if (yaw_relativo < -180.0f) yaw_relativo += 360.0f;

	yaw_filtrado = (0.25f * yaw_relativo) + (0.75f * yaw_filtrado);
	float yaw_acotado = constrain(yaw_filtrado, ANGULO_MIN_YAW, ANGULO_MAX_YAW);
	if (fabsf(yaw_acotado) <= ZONA_CERO_YAW) {
		yaw_acotado = 0.0f;
	}

	// 3. Desacoplamiento de ejes e histéresis para que M1 (Pitch) y M3 (Yaw) sean independientes
	float delta_pitch = fabsf(pitch_acotado - pitch_estable);
	float delta_yaw = fabsf(yaw_acotado - yaw_estable);

	bool pitch_a_cero = (pitch_acotado == 0.0f && pitch_estable != 0.0f);
	bool yaw_a_cero = (yaw_acotado == 0.0f && yaw_estable != 0.0f);

	if (pitch_a_cero) {
		pitch_estable = 0.0f;
	}
	if (yaw_a_cero) {
		yaw_estable = 0.0f;
	}

	if (delta_pitch >= HIST_ESTABILIDAD_PITCH && delta_pitch >= (delta_yaw * 1.15f)) {
		pitch_estable = pitch_acotado;
	} else if (delta_yaw >= HIST_ESTABILIDAD_YAW && delta_yaw > (delta_pitch * 1.15f)) {
		yaw_estable = yaw_acotado;
	} else {
		if (delta_pitch >= (HIST_ESTABILIDAD_PITCH * 1.6f)) {
			pitch_estable = pitch_acotado;
		}
		if (delta_yaw >= (HIST_ESTABILIDAD_YAW * 1.6f)) {
			yaw_estable = yaw_acotado;
		}
	}

	angulo_pitch_actual = pitch_estable;
	setpoint_brazo_pulsos = angulo_pitch_actual * PULSOS_POR_GRADO_BRAZO;

	angulo_yaw_actual = yaw_estable;
	setpoint_yaw_pulsos = angulo_yaw_actual * PULSOS_POR_GRADO_YAW;

	float proporcion = (angulo_pitch_actual - ANGULO_MIN_BRAZO) / (ANGULO_MAX_BRAZO - ANGULO_MIN_BRAZO);
	int nivel_calculado = (int)roundf(proporcion * NUM_LEDS_VIRTUALES);
	leds_encendidos_brazo = constrain(nivel_calculado, 0, NUM_LEDS_VIRTUALES);
}

static void callback_emg(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
	if (length < 16) return;

	int suma_absoluta = 0;
	for (int i = 0; i < 16; i++) {
		int8_t muestra = (int8_t)pData[i];
		suma_absoluta += abs(muestra);
	}
	float mav_actual = suma_absoluta / 16.0f;

	static float emg_suavizado = 0.0f;
	emg_suavizado = (0.22f * mav_actual) + (0.78f * emg_suavizado);
	envolvente_emg = emg_suavizado;

	if (!estado_pinza_cerrada && emg_suavizado >= umbral_apretar_pinza) {
		estado_pinza_cerrada = true;
		setpoint_pinza_pulsos = PULSOS_CIERRE_PINZA;
	} else if (estado_pinza_cerrada && emg_suavizado <= umbral_soltar_pinza) {
		estado_pinza_cerrada = false;
		setpoint_pinza_pulsos = 0.0f;
	}
}

// =========================================================================
// 7. RECEPCIÓN DE COMANDOS DESDE REACT (CAL_ZERO, SET_UMBRAL, FLIP)
// =========================================================================

void procesar_comandos_serial() {
	if (!Serial.available()) return;

	String comando = Serial.readStringUntil('\n');
	comando.trim();

	if (comando == "CAL_ZERO") {
		pitch_calibrado = false;
		pcnt_counter_clear(PCNT_M1_UNIT);
		pcnt_counter_clear(PCNT_M2_UNIT);
		pcnt_counter_clear(PCNT_M3_UNIT);
		pos_estimada_m1 = 0.0f;
		pos_estimada_m3 = 0.0f;
		posicion_acumulada_m1 = 0;
		posicion_acumulada_m2 = 0;
		posicion_acumulada_m3 = 0;
		setpoint_brazo_pulsos = 0.0f;
		setpoint_pinza_pulsos = 0.0f;
		setpoint_yaw_pulsos = 0.0f;
		pid_elevacion.reiniciar();
		pid_pinza.reiniciar();
		pid_base.reiniciar();
		solicitar_vibracion = true;
	} else if (comando == "FLIP_M1") {
		dir_encoder_m1 = -dir_encoder_m1;
		pcnt_counter_clear(PCNT_M1_UNIT);
		pos_estimada_m1 = setpoint_brazo_pulsos;
		posicion_acumulada_m1 = (int32_t)roundf(pos_estimada_m1);
		pid_elevacion.reiniciar();
	} else if (comando == "FLIP_M2") {
		dir_encoder_m2 = -dir_encoder_m2;
		pcnt_counter_clear(PCNT_M2_UNIT);
		posicion_acumulada_m2 = 0;
		pid_pinza.reiniciar();
	} else if (comando == "FLIP_M3") {
		dir_encoder_m3 = -dir_encoder_m3;
		pcnt_counter_clear(PCNT_M3_UNIT);
		pos_estimada_m3 = setpoint_yaw_pulsos;
		posicion_acumulada_m3 = (int32_t)roundf(pos_estimada_m3);
		pid_base.reiniciar();
	} else if (comando == "VIBRATE") {
		solicitar_vibracion = true;
	} else if (comando.startsWith("SET_UMBRAL:")) {
		String valores = comando.substring(11);
		int indice_coma = valores.indexOf(',');
		if (indice_coma > 0) {
			float nuevo_on = valores.substring(0, indice_coma).toFloat();
			float nuevo_off = valores.substring(indice_coma + 1).toFloat();
			if (nuevo_on > 2.0f && nuevo_off >= 1.0f && nuevo_on > nuevo_off) {
				umbral_apretar_pinza = nuevo_on;
				umbral_soltar_pinza = nuevo_off;
			}
		}
	}
}

// =========================================================================
// 8. GESTIÓN DE CONEXIÓN Y HANDSHAKE BLE CON EL MYO ARMBAND
// =========================================================================

class CallbacksClienteMyo : public BLEClientCallbacks {
	void onConnect(BLEClient* pClient) override {}

	void onDisconnect(BLEClient* pClient) override {
		myo_conectado = false;
		myo_encontrado = false;
		pitch_calibrado = false;
		pCharComandoGlobal = nullptr;
		estado_pinza_cerrada = false;
		pid_elevacion.reiniciar();
		pid_pinza.reiniciar();
		pid_base.reiniciar();
	}
};

class CallbacksEscaneoMyo : public BLEAdvertisedDeviceCallbacks {
	void onResult(BLEAdvertisedDevice advertisedDevice) override {
		if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(MYO_SVC_CONTROL)) {
			BLEDevice::getScan()->stop();
			if (myo_detectado != nullptr) {
				delete myo_detectado;
			}
			myo_detectado = new BLEAdvertisedDevice(advertisedDevice);
			myo_encontrado = true;
		}
	}
};

bool conectar_y_configurar_myo() {
	BLEClient* pClient = BLEDevice::createClient();
	pClient->setClientCallbacks(new CallbacksClienteMyo());

	if (!pClient->connect(myo_detectado)) return false;

	BLERemoteService* pSvcCtrl = pClient->getService(MYO_SVC_CONTROL);
	if (pSvcCtrl == nullptr) return false;
	pCharComandoGlobal = pSvcCtrl->getCharacteristic(MYO_CHAR_COMMAND);
	if (pCharComandoGlobal == nullptr) return false;

	uint8_t cmd_never_sleep[] = {0x09, 0x01, 0x01};
	pCharComandoGlobal->writeValue(cmd_never_sleep, sizeof(cmd_never_sleep), true);

	uint8_t cmd_unlock[] = {0x0A, 0x01, 0x02};
	pCharComandoGlobal->writeValue(cmd_unlock, sizeof(cmd_unlock), true);

	uint8_t cmd_vibrate[] = {0x03, 0x01, 0x02};
	pCharComandoGlobal->writeValue(cmd_vibrate, sizeof(cmd_vibrate), true);

	uint8_t cmd_stream[] = {0x01, 0x03, 0x02, 0x01, 0x00};
	pCharComandoGlobal->writeValue(cmd_stream, sizeof(cmd_stream), true);

	BLERemoteService* pSvcImu = pClient->getService(MYO_SVC_IMU);
	if (pSvcImu != nullptr) {
		BLERemoteCharacteristic* pCharImu = pSvcImu->getCharacteristic(MYO_CHAR_IMU_DATA);
		if (pCharImu != nullptr && pCharImu->canNotify()) {
			pCharImu->registerForNotify(callback_imu);
		}
	}

	BLERemoteService* pSvcEmg = pClient->getService(MYO_SVC_EMG);
	if (pSvcEmg != nullptr) {
		for (int i = 0; i < 4; i++) {
			BLERemoteCharacteristic* pCharEmg = pSvcEmg->getCharacteristic(BLEUUID(MYO_CHARS_EMG[i]));
			if (pCharEmg != nullptr && pCharEmg->canNotify()) {
				pCharEmg->registerForNotify(callback_emg);
			}
		}
	}

	myo_conectado = true;
	return true;
}

// =========================================================================
// 9. SETUP Y LOOP PRINCIPAL
// =========================================================================

void setup() {
	Serial.begin(115200);

	pinMode(PIN_LED_PINZA, OUTPUT);
	pinMode(PIN_LED_INTEGRADO, OUTPUT);
	digitalWrite(PIN_LED_PINZA, LOW);
	digitalWrite(PIN_LED_INTEGRADO, LOW);

	pinMode(M1_IN1_PIN, OUTPUT);
	pinMode(M1_IN2_PIN, OUTPUT);
	pinMode(M2_IN3_PIN, OUTPUT);
	pinMode(M2_IN4_PIN, OUTPUT);
	pinMode(M3_IN1_PIN, OUTPUT);
	pinMode(M3_IN2_PIN, OUTPUT);

	ledcSetup(PWM_CH_M1_IN1, PWM_FREQ, PWM_RES);
	ledcAttachPin(M1_IN1_PIN, PWM_CH_M1_IN1);

	ledcSetup(PWM_CH_M1_IN2, PWM_FREQ, PWM_RES);
	ledcAttachPin(M1_IN2_PIN, PWM_CH_M1_IN2);

	ledcSetup(PWM_CH_M2_IN3, PWM_FREQ, PWM_RES);
	ledcAttachPin(M2_IN3_PIN, PWM_CH_M2_IN3);

	ledcSetup(PWM_CH_M2_IN4, PWM_FREQ, PWM_RES);
	ledcAttachPin(M2_IN4_PIN, PWM_CH_M2_IN4);

	ledcSetup(PWM_CH_M3_IN1, PWM_FREQ, PWM_RES);
	ledcAttachPin(M3_IN1_PIN, PWM_CH_M3_IN1);

	ledcSetup(PWM_CH_M3_IN2, PWM_FREQ, PWM_RES);
	ledcAttachPin(M3_IN2_PIN, PWM_CH_M3_IN2);

	inicializar_pcnt_x4(PCNT_M1_UNIT, ENC1_C1_PIN, ENC1_C2_PIN);
	inicializar_pcnt_x4(PCNT_M2_UNIT, ENC2_C1_PIN, ENC2_C2_PIN);
	inicializar_pcnt_x4(PCNT_M3_UNIT, ENC3_C1_PIN, ENC3_C2_PIN);

	xTaskCreatePinnedToCore(tarea_lazo_cerrado_pid, "PID_Motors", 4096, NULL, 3, NULL, 1);

	BLEDevice::init("ESP32_Myo_3DOF");
	BLEScan* pScan = BLEDevice::getScan();
	pScan->setAdvertisedDeviceCallbacks(new CallbacksEscaneoMyo());
	pScan->setInterval(45);
	pScan->setWindow(15);
	pScan->setActiveScan(true);
	pScan->start(5, false);
}

void loop() {
	procesar_comandos_serial();

	if (myo_encontrado && !myo_conectado) {
		if (!conectar_y_configurar_myo()) {
			myo_encontrado = false;
		}
	}

	if (myo_conectado) {
		if (solicitar_vibracion && pCharComandoGlobal != nullptr) {
			uint8_t cmd_vib_corta[] = {0x03, 0x01, 0x01};
			pCharComandoGlobal->writeValue(cmd_vib_corta, sizeof(cmd_vib_corta), false);
			solicitar_vibracion = false;
		}

		digitalWrite(PIN_LED_PINZA, estado_pinza_cerrada ? HIGH : LOW);
		digitalWrite(PIN_LED_INTEGRADO, estado_pinza_cerrada ? HIGH : LOW);

		Serial.printf(
			"{\"conectado\":true,\"pitch\":%.2f,\"yaw\":%.2f,\"leds\":%d,\"emg\":%.2f,\"uOn\":%.1f,\"uOff\":%.1f,\"pinza\":%d,\"posM1\":%ld,\"spM1\":%.1f,\"posM2\":%ld,\"spM2\":%.1f,\"posM3\":%ld,\"spM3\":%.1f}\n",
			angulo_pitch_actual,
			angulo_yaw_actual,
			leds_encendidos_brazo,
			envolvente_emg,
			umbral_apretar_pinza,
			umbral_soltar_pinza,
			estado_pinza_cerrada ? 1 : 0,
			(long)posicion_acumulada_m1,
			setpoint_brazo_pulsos,
			(long)posicion_acumulada_m2,
			setpoint_pinza_pulsos,
			(long)posicion_acumulada_m3,
			setpoint_yaw_pulsos
		);

		vTaskDelay(pdMS_TO_TICKS(30));
	} else {
		static bool blink = false;
		blink = !blink;
		digitalWrite(PIN_LED_INTEGRADO, blink ? HIGH : LOW);
		digitalWrite(PIN_LED_PINZA, LOW);

		Serial.println("{\"conectado\":false}");
		BLEDevice::getScan()->start(1, false);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

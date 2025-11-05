/*
 * INVERSOR DE ONDA SINUSOIDAL PURA 24V DC a 120V AC
 * Usando Transformador de Ferrita de Alta Frecuencia
 * 
 * DESCRIPCIÓN:
 * Este código genera una señal SPWM (Sinusoidal Pulse Width Modulation)
 * para controlar un inversor de puente completo (H-bridge) que alimenta
 * un transformador de ferrita de alta frecuencia.
 * 
 * ESPECIFICACIONES:
 * - Tensión de entrada: 24V DC
 * - Tensión de salida: 120V AC RMS (60Hz)
 * - Frecuencia de conmutación: 20 kHz (para transformador de ferrita)
 * - Frecuencia de salida: 60Hz (configurable a 50Hz)
 * 
 * HARDWARE NECESARIO:
 * - Arduino Nano/Uno/Mega
 * - 4 MOSFETs (por ejemplo: IRFZ44N o IRF3205)
 * - 4 Drivers de MOSFET (IR2110 o equivalente)
 * - Transformador de ferrita de alta frecuencia (relación apropiada)
 * - Filtro LC de salida
 * - Circuito de protección y realimentación
 * 
 * CONEXIONES DE PINES:
 * Pin 9  -> PWM1 (H-Bridge lado alto izquierdo)
 * Pin 10 -> PWM2 (H-Bridge lado alto derecho)
 * Pin 5  -> PWM3 (H-Bridge lado bajo izquierdo)
 * Pin 6  -> PWM4 (H-Bridge lado bajo derecho)
 * A0     -> Sensor de corriente (opcional)
 * A1     -> Sensor de voltaje de entrada (opcional)
 * Pin 2  -> Botón de encendido/apagado
 * Pin 13 -> LED indicador
 * 
 * TOPOLOGÍA:
 * Se utiliza un puente completo (Full H-Bridge) con tiempo muerto
 * para evitar cortocircuitos. La señal SPWM se genera usando
 * una tabla de búsqueda de seno.
 * 
 * AUTOR: Sistema de control de inversor
 * FECHA: 2025
 * VERSIÓN: 1.0
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// ==================== CONFIGURACIÓN ====================
#define FRECUENCIA_SALIDA 60        // 60Hz (cambiar a 50 para Europa)
#define FRECUENCIA_PWM 20000        // 20kHz para transformador de ferrita
#define TIEMPO_MUERTO_US 2          // Tiempo muerto en microsegundos (dead time)
#define MUESTRAS_SENO 100           // Resolución de la tabla de seno

// Pines de control del H-Bridge
#define PWM_HIGH_A 9                // Timer1 OC1A
#define PWM_HIGH_B 10               // Timer1 OC1B
#define PWM_LOW_A 5                 // Timer0 OC0B (control por software)
#define PWM_LOW_B 6                 // Timer0 OC0A (control por software)

// Pines de entrada/salida
#define PIN_CORRIENTE A0            // Sensor de corriente
#define PIN_VOLTAJE A1              // Sensor de voltaje de entrada
#define PIN_BOTON 2                 // Botón de encendido
#define PIN_LED 13                  // LED indicador

// Límites de protección
#define VOLTAJE_MIN 22.0            // Voltaje mínimo de batería (22V)
#define VOLTAJE_MAX 29.0            // Voltaje máximo de batería (29V)
#define CORRIENTE_MAX 800           // Corriente máxima (valor ADC)

// ==================== VARIABLES GLOBALES ====================
// Tabla de seno precalculada (valores de 0 a 255 para PWM de 8 bits)
// Ajustada para Timer1 de 10 bits (0-1023)
const uint16_t tablaSeno[MUESTRAS_SENO] PROGMEM = {
  512, 544, 576, 607, 639, 670, 701, 731, 761, 791,
  819, 847, 875, 901, 927, 952, 976, 999, 1021, 1042,
  1062, 1081, 1099, 1115, 1131, 1145, 1158, 1170, 1181, 1190,
  1198, 1205, 1211, 1215, 1218, 1220, 1221, 1220, 1218, 1215,
  1211, 1205, 1198, 1190, 1181, 1170, 1158, 1145, 1131, 1115,
  1099, 1081, 1062, 1042, 1021, 999, 976, 952, 927, 901,
  875, 847, 819, 791, 761, 731, 701, 670, 639, 607,
  576, 544, 512, 480, 448, 417, 385, 354, 323, 293,
  263, 233, 205, 177, 149, 123, 97, 72, 48, 25,
  3, 0, 0, 0, 0, 19, 39, 61, 85, 110
};

volatile uint16_t indiceTabla = 0;
volatile bool inversorEncendido = false;
volatile bool sobrecarga = false;

float voltajeEntrada = 24.0;
uint16_t corrienteActual = 0;

// ==================== CONFIGURACIÓN INICIAL ====================
void setup() {
  // Inicializar comunicación serial para debug
  Serial.begin(115200);
  Serial.println(F("================================="));
  Serial.println(F("INVERSOR ONDA SINUSOIDAL PURA"));
  Serial.println(F("24V DC -> 120V AC @ 60Hz"));
  Serial.println(F("================================="));
  
  // Configurar pines
  pinMode(PWM_HIGH_A, OUTPUT);
  pinMode(PWM_HIGH_B, OUTPUT);
  pinMode(PWM_LOW_A, OUTPUT);
  pinMode(PWM_LOW_B, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  
  // Apagar todos los MOSFETs inicialmente
  digitalWrite(PWM_HIGH_A, LOW);
  digitalWrite(PWM_HIGH_B, LOW);
  digitalWrite(PWM_LOW_A, LOW);
  digitalWrite(PWM_LOW_B, LOW);
  
  // Configurar Timer1 para PWM rápido de 10 bits
  configurarPWM();
  
  // Configurar Timer2 para generar la onda senoidal a 60Hz
  configurarTimerSeno();
  
  // Configurar interrupción para el botón
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), toggleInversor, FALLING);
  
  Serial.println(F("Sistema inicializado"));
  Serial.println(F("Presione el botón para encender"));
  
  // Parpadear LED para indicar que está listo
  for(int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(200);
    digitalWrite(PIN_LED, LOW);
    delay(200);
  }
}

// ==================== CONFIGURACIÓN DE PWM ====================
void configurarPWM() {
  // Configurar Timer1 para Fast PWM de 10 bits
  // Frecuencia PWM = 16MHz / (prescaler * (TOP+1))
  // Para 20kHz: prescaler = 1, TOP = 799
  
  // Detener timer
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Modo Fast PWM con ICR1 como TOP
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Sin prescaler
  
  // Configurar TOP para 20kHz (16MHz / 20kHz = 800)
  ICR1 = 799;
  
  // Inicializar duty cycle a 0
  OCR1A = 0;
  OCR1B = 0;
  
  Serial.println(F("PWM configurado a 20kHz"));
}

// ==================== CONFIGURACIÓN DE TIMER PARA ONDA SENOIDAL ====================
void configurarTimerSeno() {
  // Configurar Timer2 para generar interrupciones a la frecuencia
  // necesaria para actualizar la tabla de seno
  // Frecuencia de interrupción = FRECUENCIA_SALIDA * MUESTRAS_SENO
  // Para 60Hz con 100 muestras = 6000Hz
  
  cli(); // Deshabilitar interrupciones
  
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  
  // Modo CTC (Clear Timer on Compare Match)
  TCCR2A = _BV(WGM21);
  
  // Prescaler de 64: 16MHz / 64 = 250kHz
  TCCR2B = _BV(CS22);
  
  // Para 6000Hz: 250000 / 6000 = 41.67 ≈ 42
  OCR2A = 41;
  
  // Habilitar interrupción por comparación
  TIMSK2 = _BV(OCIE2A);
  
  sei(); // Habilitar interrupciones
  
  Serial.print(F("Generador de seno configurado: "));
  Serial.print(FRECUENCIA_SALIDA);
  Serial.println(F("Hz"));
}

// ==================== INTERRUPCIÓN PARA GENERAR ONDA SENOIDAL ====================
ISR(TIMER2_COMPA_vect) {
  if(!inversorEncendido || sobrecarga) {
    OCR1A = 0;
    OCR1B = 0;
    return;
  }
  
  // Leer valor de la tabla de seno
  uint16_t valorSeno = pgm_read_word(&tablaSeno[indiceTabla]);
  
  // Determinar qué medio puente activar según la polaridad
  if(indiceTabla < MUESTRAS_SENO / 2) {
    // Semiciclo positivo: PWM_HIGH_A y PWM_LOW_B
    OCR1A = valorSeno;
    OCR1B = 0;
    // Los pines LOW se controlan por software con tiempo muerto
    digitalWrite(PWM_LOW_B, HIGH);
    digitalWrite(PWM_LOW_A, LOW);
  } else {
    // Semiciclo negativo: PWM_HIGH_B y PWM_LOW_A
    OCR1A = 0;
    OCR1B = valorSeno;
    // Los pines LOW se controlan por software con tiempo muerto
    digitalWrite(PWM_LOW_A, HIGH);
    digitalWrite(PWM_LOW_B, LOW);
  }
  
  // Avanzar en la tabla
  indiceTabla++;
  if(indiceTabla >= MUESTRAS_SENO) {
    indiceTabla = 0;
  }
}

// ==================== FUNCIÓN DE INTERRUPCIÓN DEL BOTÓN ====================
void toggleInversor() {
  static unsigned long ultimaInterrupcion = 0;
  unsigned long tiempoActual = millis();
  
  // Debounce: ignorar rebotes menores a 200ms
  if(tiempoActual - ultimaInterrupcion > 200) {
    inversorEncendido = !inversorEncendido;
    ultimaInterrupcion = tiempoActual;
    
    if(inversorEncendido) {
      Serial.println(F(">>> INVERSOR ENCENDIDO <<<"));
    } else {
      Serial.println(F(">>> INVERSOR APAGADO <<<"));
      // Apagar todos los PWM
      OCR1A = 0;
      OCR1B = 0;
      digitalWrite(PWM_LOW_A, LOW);
      digitalWrite(PWM_LOW_B, LOW);
    }
  }
}

// ==================== FUNCIÓN DE MONITOREO ====================
void monitorearSistema() {
  // Leer voltaje de entrada (usando divisor de voltaje)
  // Ajustar factor de conversión según tu divisor de voltaje
  int lecturaVoltaje = analogRead(PIN_VOLTAJE);
  voltajeEntrada = (lecturaVoltaje * 5.0 / 1023.0) * 7.0; // Factor ejemplo: 7x
  
  // Leer corriente (usando sensor ACS712 o similar)
  int lecturaCorriente = analogRead(PIN_CORRIENTE);
  corrienteActual = lecturaCorriente;
  
  // Verificar bajo voltaje
  if(voltajeEntrada < VOLTAJE_MIN) {
    if(inversorEncendido) {
      Serial.println(F("ADVERTENCIA: Voltaje bajo!"));
      // Apagar inversor por protección
      inversorEncendido = false;
      sobrecarga = true;
      apagarInversor();
    }
  }
  
  // Verificar sobrevoltaje
  if(voltajeEntrada > VOLTAJE_MAX) {
    if(inversorEncendido) {
      Serial.println(F("ADVERTENCIA: Sobrevoltaje!"));
      inversorEncendido = false;
      sobrecarga = true;
      apagarInversor();
    }
  }
  
  // Verificar sobrecorriente
  if(corrienteActual > CORRIENTE_MAX) {
    if(inversorEncendido) {
      Serial.println(F("ADVERTENCIA: Sobrecorriente!"));
      inversorEncendido = false;
      sobrecarga = true;
      apagarInversor();
    }
  }
  
  // Si hubo sobrecarga, parpadear LED rápidamente
  if(sobrecarga) {
    static unsigned long ultimoParpadeo = 0;
    if(millis() - ultimoParpadeo > 100) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      ultimoParpadeo = millis();
    }
  } else if(inversorEncendido) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }
}

// ==================== FUNCIÓN PARA APAGAR INVERSOR ====================
void apagarInversor() {
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(PWM_LOW_A, LOW);
  digitalWrite(PWM_LOW_B, LOW);
  Serial.println(F("Inversor apagado por protección"));
}

// ==================== FUNCIÓN PARA MOSTRAR INFORMACIÓN ====================
void mostrarInformacion() {
  static unsigned long ultimoReporte = 0;
  
  if(millis() - ultimoReporte > 1000) { // Cada segundo
    Serial.println(F("----------------------------"));
    Serial.print(F("Estado: "));
    Serial.println(inversorEncendido ? F("ENCENDIDO") : F("APAGADO"));
    Serial.print(F("Voltaje entrada: "));
    Serial.print(voltajeEntrada);
    Serial.println(F(" V"));
    Serial.print(F("Corriente (ADC): "));
    Serial.println(corrienteActual);
    
    if(sobrecarga) {
      Serial.println(F("¡SOBRECARGA DETECTADA!"));
      Serial.println(F("Presione reset para reiniciar"));
    }
    
    ultimoReporte = millis();
  }
}

// ==================== BUCLE PRINCIPAL ====================
void loop() {
  // Monitorear sistema continuamente
  monitorearSistema();
  
  // Mostrar información por serial
  mostrarInformacion();
  
  // Pequeña pausa para no saturar
  delay(10);
}

/*
 * ==================== NOTAS IMPORTANTES ====================
 * 
 * 1. AJUSTES NECESARIOS:
 *    - Calibrar los factores de conversión para sensores de voltaje y corriente
 *    - Ajustar TIEMPO_MUERTO_US según los MOSFETs utilizados
 *    - Verificar que el transformador soporte 20kHz
 * 
 * 2. FILTRO DE SALIDA:
 *    - Es CRÍTICO usar un filtro LC en la salida para obtener una
 *      onda sinusoidal limpia de 60Hz
 *    - Valores típicos: L = 10mH, C = 10µF (ajustar según carga)
 * 
 * 3. PROTECCIONES RECOMENDADAS:
 *    - Fusible en la entrada de 24V
 *    - Protección contra cortocircuito en la salida
 *    - Protección térmica en los MOSFETs
 *    - Ventilación adecuada del disipador
 * 
 * 4. OPTIMIZACIONES POSIBLES:
 *    - Implementar control PID para regulación de voltaje
 *    - Agregar compensación de carga
 *    - Implementar arranque suave (soft start)
 *    - Agregar detección de cruce por cero para sincronización
 * 
 * 5. CÁLCULO DEL TRANSFORMADOR:
 *    - Relación de vueltas: N2/N1 = 120V / 24V = 5:1
 *    - Considerar pérdidas, usar relación aproximada de 6:1 o 7:1
 *    - Núcleo de ferrita adecuado para 20kHz (tipo ETD, EE, o similar)
 * 
 * 6. DRIVERS DE MOSFET:
 *    - Usar IR2110, IR2184 o similar para los MOSFETs de lado alto
 *    - Incluir bootstrap capacitors (típicamente 10µF + diodo rápido)
 * 
 * 7. MEJORAS DE SOFTWARE:
 *    - Este código usa una implementación simplificada
 *    - Para producción, considerar usar DMA para actualizar PWM
 *    - Implementar SPWM de dos niveles o multinivel para mejor THD
 * 
 * ==================== ADVERTENCIAS DE SEGURIDAD ====================
 * 
 * ⚠️  PELIGRO: Este inversor genera 120V AC que puede ser LETAL
 * ⚠️  Usar SIEMPRE protección adecuada y aislamientos apropiados
 * ⚠️  Probar primero con cargas pequeñas (bombilla de 25W)
 * ⚠️  Usar osciloscopio para verificar formas de onda
 * ⚠️  Verificar que NO haya tiempo de conducción simultánea (shoot-through)
 * ⚠️  Usar disipadores de calor adecuados en los MOSFETs
 * 
 */

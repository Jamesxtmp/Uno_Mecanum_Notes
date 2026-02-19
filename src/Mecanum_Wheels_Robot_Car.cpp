#include <Arduino.h>

// --- Tone detection globals ---
int   in[128];
byte NoteV[13]={8,23,40,57,76,96,116,138,162,187,213,241,255};
float f_peaks[5]; // top 5 frequencies
int Mic_pin = A4;  // microphone analog pin
int detectedNote = -1; // -1 none, 0..11 notes C..B
// Behavior flags / sampling
const bool DEBUG_NOTES = false; // set true to print every detected note
const bool VERBOSE = false; // true = mostrar info detallada, false = silencioso (más rápido)
const int SAMPLE_SIZE = 5; // number of consecutive detections to sample

// Forward declarations
void Tone_det();
void FFT(byte N,float Frequency);

volatile int Front_Distance;
volatile boolean Flag=true;

const int Trig = A3;
const int Echo= A2; 
const int PWM2A = 11;      //M1 motor
const int PWM2B = 3;       //M2 motor  
const int PWM0A = 6;       //M3 motor 
const int PWM0B = 5;       //M4 motor
const int DIR_CLK = 4;     // Data input clock line
const int DIR_EN = 7;      //Equip the L293D enabling pins
const int DATA = 8;        // USB cable
const int DIR_LATCH = 12;  // Output memory latch clock
//Define the pin of ultrasonic obstacle avoidance sensor
//Define motion state
const int Move_Forward = 39;    //Move Forward
const int Move_Backward = 216;      //Move Backward
const int Left_Move= 116;      //Left translation
const int Right_Move = 139;     //Right translation
const int Right_Rotate = 149;     //Rotate Left
const int Left_Rotate = 106;     //Rotate Left
const int Stop = 0;        //Parking variable
const int Upper_Left_Move = 36;    //Upper Left Move
const int Upper_Right_Move = 3;    //Upper Right Move
const int Lower_Left_Move = 80;    //Lower Left Move
const int Lower_Right_Move = 136;    //Lower Right Move
const int Drift_Left = 20;    //Drift on Left
const int Drift_Right = 10;    //Drift on Right
//Set the default speed between 1 and 255
int Speed1 = 255;
int Speed2 = 255;
int Speed3 = 255;
int Speed4 = 255;

void Motor(int Dir,int S1,int S2,int S3,int S4)
{
    analogWrite(PWM2A,S1); //Motor PWM speed regulation
    analogWrite(PWM2B,S2); //Motor PWM speed regulation
    analogWrite(PWM0A,S3); //Motor PWM speed regulation
    analogWrite(PWM0B,S4); //Motor PWM speed regulation
    
    digitalWrite(DIR_LATCH,LOW); //DIR_LATCH sets the low level and writes the direction of motion in preparation
    shiftOut(DATA,DIR_CLK,MSBFIRST,Dir);//Write Dir motion direction value
    digitalWrite(DIR_LATCH,HIGH);//DIR_LATCH sets the high level and outputs the direction of motion
}

float checkdistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float distance = pulseIn(Echo, HIGH) / 58.00;
  delay(10);
  return distance;
}

void Ultrasonic_Avoidance() 
{
  int Front_Distance=0;
  Front_Distance = checkdistance();
  if (0 < Front_Distance && Front_Distance <= 45) {
    if (Front_Distance<=20) {
      Flag=!Flag;
      Motor(Stop,0,0,0,0);
      delay(250);
      Motor(Move_Backward,Speed1,Speed2,Speed3,Speed4);
      delay(200);
      Motor(Stop,0,0,0,0);
      delay(250);
      if (Flag) 
      {
      Motor(Left_Rotate,Speed1,Speed2,Speed3,Speed4);
      } else 
      {
      Motor(Right_Rotate,Speed1,Speed2,Speed3,Speed4);
      }   
      delay(100);
      Motor(Stop,0,0,0,0);
      delay(250);

    } else {
      Motor(Stop,0,0,0,0);
      delay(250);
      if (Flag) 
      {
      Motor(Left_Rotate,Speed1,Speed2,Speed3,Speed4);
      } else 
      {
      Motor(Right_Rotate,Speed1,Speed2,Speed3,Speed4);
      }
      delay(100);
      Motor(Stop,0,0,0,0);
      delay(250); 
    }

  } else {
    Motor(Move_Forward,100,100,100,100);

  }
}


void Ultrasonic_Follow() 
{
  Front_Distance = checkdistance();
  if ( (Front_Distance >=0)&&(Front_Distance <= 10)) 
  {
    Motor(Move_Backward,Speed1,Speed2,Speed3,Speed4);
    delay(20);
  } else if ( (Front_Distance > 10) && (Front_Distance <= 15) ) {
    Motor(Stop,0,0,0,0);
    delay(20);
  } 
  else
  {
    Motor(Move_Forward,170,170,170,170);
    delay(20);
  }

}
void setup(){

  Front_Distance = 0;
  detectedNote = -1;

  Serial.begin(9600);//Serial for debug
  //Configure as output mode
  pinMode(DIR_CLK,OUTPUT);
  pinMode(DATA,OUTPUT);
  pinMode(DIR_EN,OUTPUT);
  pinMode(DIR_LATCH,OUTPUT);
  pinMode(PWM0B,OUTPUT);
  pinMode(PWM0A,OUTPUT);
  pinMode(PWM2A,OUTPUT);
  pinMode(PWM2B,OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop(){
  // Take SAMPLE_SIZE detections and pick the most frequent note
  int notes[SAMPLE_SIZE];
  
  for (int i=0;i<SAMPLE_SIZE;i++) {
    Tone_det();
    notes[i] = detectedNote;
    delay(0); // Pausa entre muestras - reducido para mayor velocidad
  }

  // Nombres de notas para mostrar
  const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  
  // Count occurrences of each note (0..11)
  int counts[12] = {0};
  for (int i=0;i<SAMPLE_SIZE;i++) {
    int n = notes[i];
    if (n >=0 && n <=11) counts[n]++;
  }

  // Find most frequent note
  int maxCount = 0;
  int maxNote = -1;
  for (int n=0;n<12;n++) {
    if (counts[n] > maxCount) { maxCount = counts[n]; maxNote = n; }
  }

  // Required count = al menos 4 de 5 muestras (80%)
  int required = 4;

  // Compute median (mediana) of valid sampled notes - más robusto contra outliers que el promedio
  int validNotesArray[SAMPLE_SIZE];
  int validNotes = 0;
  for (int i=0;i<SAMPLE_SIZE;i++) {
    if (notes[i] >= 0 && notes[i] <= 11) { 
      validNotesArray[validNotes] = notes[i];
      validNotes++;
    }
  }
  
  int median = -1;
  if (validNotes > 0) {
    // Simple bubble sort para ordenar las notas válidas
    for (int i=0; i<validNotes-1; i++) {
      for (int j=0; j<validNotes-i-1; j++) {
        if (validNotesArray[j] > validNotesArray[j+1]) {
          int temp = validNotesArray[j];
          validNotesArray[j] = validNotesArray[j+1];
          validNotesArray[j+1] = temp;
        }
      }
    }
    // Tomar el valor del medio (mediana)
    median = validNotesArray[validNotes/2];
  }

  // Validar: frecuencia suficiente Y mediana igual a la nota más frecuente
  bool isStable = false;
  if (maxCount >= required && maxNote >= 0) {
    if (median == maxNote) {  // La mediana debe coincidir exactamente con la nota más frecuente
      isStable = true;
    }
  }

  // Decidir la nota detectada
  if (isStable) {
    detectedNote = maxNote;
  } else {
    detectedNote = -1;
  }

  // Mostrar información en formato compacto
  if (VERBOSE) {
    Serial.println("===============================");
    Serial.print("Notas: [ ");
    for (int i=0;i<SAMPLE_SIZE;i++) {
      if (notes[i] >= 0 && notes[i] <= 11) {
        Serial.print(noteNames[notes[i]]);
      } else {
        Serial.print("-");
      }
      if (i < SAMPLE_SIZE-1) Serial.print(", ");
    }
    Serial.println(" ]");
    
    Serial.print("Frecuente (");
    Serial.print(maxCount);
    Serial.print("/");
    Serial.print(SAMPLE_SIZE);
    Serial.print(") > ");
    if (isStable && maxNote >= 0) {
      Serial.print(noteNames[maxNote]);
    } else {
      Serial.print("X");
    }
    Serial.println(" <");
    Serial.println("==============================");
  }

  // Map the (stable) detectedNote to movement
  // Activar el motor brevemente y luego detenerlo
  // Solo notas principales - los sostenidos quedan como margen de error
  switch(detectedNote) {
    case 0: // C - Adelante
      Motor(Move_Forward,Speed1,Speed2,Speed3,Speed4);
      delay(150);  // Pulso de movimiento
      Motor(Stop,0,0,0,0);
      break;
    case 1: // C# - Margen de error (Stop)
      Motor(Stop,0,0,0,0);
      break;
    case 2: // D - Atrás
      Motor(Move_Backward,Speed1,Speed2,Speed3,Speed4);
      delay(150);
      Motor(Stop,0,0,0,0);
      break;
    case 3: // D# - Margen de error (Stop)
      Motor(Stop,0,0,0,0);
      break;
    case 4: // E - Izquierda
      Motor(Left_Move,Speed1,Speed2,Speed3,Speed4);
      delay(150);
      Motor(Stop,0,0,0,0);
      break;
    case 5: // F - Derecha
      Motor(Right_Move,Speed1,Speed2,Speed3,Speed4);
      delay(150);
      Motor(Stop,0,0,0,0);
      break;
    case 6: // F# - Margen de error (Stop)
      Motor(Stop,0,0,0,0);
      break;
    case 7: // G - Giro izquierda
      Motor(Left_Rotate,Speed1,Speed2,Speed3,Speed4);
      delay(320);
      Motor(Stop,0,0,0,0);
      break;
    case 8: // G# - Margen de error (Stop)
      Motor(Stop,0,0,0,0);
      break;
    case 9: // A - Giro derecha
      Motor(Right_Rotate,Speed1,Speed2,Speed3,Speed4);
      delay(320);
      Motor(Stop,0,0,0,0);
      break;
    case 10: // A# - Margen de error (Stop)
      Motor(Stop,0,0,0,0);
      break;
    case 11: // B - Margen de error (Stop)
      Motor(Stop,0,0,0,0);
      break;
    default:
      Motor(Stop,0,0,0,0);
      break;
  }

  // Esperar a que el ruido del motor se disipe antes del siguiente muestreo
  // Solo si se activó un movimiento (notas activas: C, D, E, F, G, A)
  if (detectedNote == 0 || detectedNote == 2 || detectedNote == 4 || 
      detectedNote == 5 || detectedNote == 7 || detectedNote == 9) {
    delay(50);  // Tiempo para que las vibraciones se disipen
  }

}

//-----------------------------Tone Detection Function----------------------------------------------//
// Original code adapted to set `detectedNote` (0=C .. 11=B)

void Tone_det()
{   long unsigned int a1,b,a2;
  float a;
  float sum1=0,sum2=0;
  float sampling;
  detectedNote = -1; // Inicializar por defecto
  a1=micros();
        for(int i=0;i<128;i++)
          {
            a=analogRead(Mic_pin)-500;      //rough zero shift
            //utilising time between two sample for windowing   & amplitude calculation
            sum1=sum1+a;              //to average value
             sum2=sum2+a*a;            // to RMS value
            a=a*(sin(i*3.14/128)*sin(i*3.14/128));    // Hann window
            in[i]=10*a;                // scaling for float   to int conversion
            delayMicroseconds(195);   // based on operation   frequency range
          }
b=micros();
sum2=sqrt(sum2/128);         // RMS
sampling= 128000000/(b-a1);   // real time sampling frequency

if(sum2-sum1>3){  
        FFT(128,sampling);

 for(int i=0;i<12;i++){in[i]=0;}  // utilising in[] array for further   calculation

int j=0,k=0; //below loop will convert frequency value to note   
       for(int i=0; i<5;i++)
           {
           if(f_peaks[i]>1040){f_peaks[i]=0;}
            if(f_peaks[i]>=65.4   && f_peaks[i]<=130.8) {f_peaks[i]=255*((f_peaks[i]/65.4)-1);}
            if(f_peaks[i]>=130.8  && f_peaks[i]<=261.6) {f_peaks[i]=255*((f_peaks[i]/130.8)-1);}
            if(f_peaks[i]>=261.6  && f_peaks[i]<=523.25){f_peaks[i]=255*((f_peaks[i]/261.6)-1);}
            if(f_peaks[i]>=523.25 && f_peaks[i]<=1046)  {f_peaks[i]=255*((f_peaks[i]/523.25)-1);}
            if(f_peaks[i]>=1046 && f_peaks[i]<=2093)  {f_peaks[i]=255*((f_peaks[i]/1046)-1);}
            if(f_peaks[i]>255){f_peaks[i]=254;}
           j=1;k=0;
         
          while(j==1)
              {
              if(f_peaks[i]<NoteV[k]){f_peaks[i]=k;j=0;}
               k++;  // a note with max peaks (harmonic) with aplitude priority is   selected
              if(k>15){j=0;}
              }

              if(f_peaks[i]==12){f_peaks[i]=0;}
               k=f_peaks[i];
              in[k]=in[k]+(5-i);
            }

k=0;j=0;
           for(int i=0;i<12;i++)
             {
              if(k<in[i]){k=in[i];j=i;}   //Max value detection
             }
        k=j;
        detectedNote = k;
        // Print note for debug
        switch(k) {
          case 0: if (DEBUG_NOTES) Serial.println("C"); break;
          case 1: if (DEBUG_NOTES) Serial.println("C#"); break;
          case 2: if (DEBUG_NOTES) Serial.println("D"); break;
          case 3: if (DEBUG_NOTES) Serial.println("D#"); break;
          case 4: if (DEBUG_NOTES) Serial.println("E"); break;
          case 5: if (DEBUG_NOTES) Serial.println("F"); break;
          case 6: if (DEBUG_NOTES) Serial.println("F#"); break;
          case 7: if (DEBUG_NOTES) Serial.println("G"); break;
          case 8: if (DEBUG_NOTES) Serial.println("G#"); break;
          case 9: if (DEBUG_NOTES) Serial.println("A"); break;
          case 10: if (DEBUG_NOTES) Serial.println("A#"); break;
          case 11: if (DEBUG_NOTES) Serial.println("B"); break;
          default: if (DEBUG_NOTES) Serial.println("-"); detectedNote = -1; break;
        }
       }
}

//-----------------------------FFT Function----------------------------------------------//
void   FFT(byte N,float Frequency)
{
byte data[8]={1,2,4,8,16,32,64,128};
int   a,c1,f,o,x;
a=N;  
                                 
      for(int i=0;i<8;i++)                  //calculating the levels
         { if(data[i]<=a){o=i;} }
       o=7;
byte in_ps[data[o]]={};     //input for sequencing
float out_r[data[o]]={};    //real part of transform
float out_im[data[o]]={};  //imaginory part of transform
            
x=0;  
      for(int b=0;b<o;b++)                     // bit reversal
          {
          c1=data[b];
          f=data[o]/(c1+c1);
                for(int   j=0;j<c1;j++)
                    { 
                     x=x+1;
                     in_ps[x]=in_ps[j]+f;
                     }
         }
 
      for(int i=0;i<data[o];i++)            //   update input array as per bit reverse order
         {
          if(in_ps[i]<a)
           {out_r[i]=in[in_ps[i]];}
          if(in_ps[i]>a)
          {out_r[i]=in[in_ps[i]-a];}       
         }

int i10,i11,n1;
float e,c,s,tr,ti;

    for(int   i=0;i<o;i++)                                    //fft
    {
     i10=data[i];               // overall values of sine cosine  
     i11=data[o]/data[i+1];     // loop with similar sine cosine
     e=6.283/data[i+1];
     e=0-e;
      n1=0;

          for(int j=0;j<i10;j++)
          {
          c=cos(e*j);   
          s=sin(e*j); 
          n1=j;
          
                for(int   k=0;k<i11;k++)
                 {
                 tr=c*out_r[i10+n1]-s*out_im[i10+n1];
                  ti=s*out_r[i10+n1]+c*out_im[i10+n1];
          
                 out_r[n1+i10]=out_r[n1]-tr;
                  out_r[n1]=out_r[n1]+tr;
          
                 out_im[n1+i10]=out_im[n1]-ti;
                  out_im[n1]=out_im[n1]+ti;          
          
                 n1=n1+i10+i10;
                   }       
             }
     }

//---> here onward   out_r contains amplitude and our_in conntains frequency (Hz)
    for(int i=0;i<data[o-1];i++)                // getting amplitude from compex number
        {
         out_r[i]=sqrt((out_r[i]*out_r[i])+(out_im[i]*out_im[i]));   // to  increase the speed delete sqrt
         out_im[i]=(i*Frequency)/data[o];
        }

x=0;       // peak detection
   for(int i=1;i<data[o-1]-1;i++)
       {
      if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) 
      {in_ps[x]=i;     //in_ps array used for storage of peak number
      x=x+1;}    
      }

s=0;
c=0;
     for(int i=0;i<x;i++)             // re arraange as per magnitude
    {
         for(int j=c;j<x;j++)
        {
            if(out_r[in_ps[i]]<out_r[in_ps[j]])   
                {s=in_ps[i];
                in_ps[i]=in_ps[j];
                in_ps[j]=s;}
         }
    c=c+1;
    }
    
    for(int i=0;i<5;i++)     // updating   f_peak array (global variable)with descending order
     {
     f_peaks[i]=(out_im[in_ps[i]-1]*out_r[in_ps[i]-1]+out_im[in_ps[i]]*out_r[in_ps[i]]+out_im[in_ps[i]+1]*out_r[in_ps[i]+1])
      /(out_r[in_ps[i]-1]+out_r[in_ps[i]]+out_r[in_ps[i]+1]);
     }
}

#include <SoftwareSerial.h>
#include <Wire.h> // ספריה לתקשורת I2C (בשביל הגירוסקופ)

// --- חיבורי מנועי נסיעה (לדרייבר L298N הראשי) ---
// מנוע שמאל
#define LEFT_ENA       12 //לPWM כדי לשלוט במהירות
#define LEFT_IN1       11 // כיוון קדימה
#define LEFT_IN2       10 // כיוון אחורה

// מנוע ימין
#define RIGHT_ENB      13 // פין PWM למהירות מנוע ימין
#define RIGHT_IN3      15 
#define RIGHT_IN4      14 

// --- חיבורי מערכת הרמה (אנקודר + מנוע DC) ---
// חובה לחבר את האנקודר לפינים שתומכים בפסיקות חומרה (Interrupts)
#define ENCODER_CLK_A  2  // פסיקה מספר 0 במגה
#define ENCODER_DT_B   3  // פסיקה מספר 1 במגה

#define DC_LIFT_ENA    4  // PWM למנוע שמוריד/מעלה את הדיזה
#define DC_LIFT_IN1    5  
#define DC_LIFT_IN2    6  

// --- מערכת צביעה (שסתום חשמלי) ---
#define VALVE_PIN 32 // מחובר למודול ממסר שפותח את האיירלס
#define VALVE_OPEN HIGH 
#define VALVE_CLOSED LOW 

// --- חיישנים אולטרסוניים (נסיעה ויישור) ---
// העברתי את כולם לפינים 24-31 (הפינים הגבוהים במגה) כדי שלא יתנגשו 
// בטעות עם הפסיקות או ה-PWM של המנועים.
#define FRONT_TRIG_PIN 24 // חיישן חזית - לזיהוי הקיר ממול
#define FRONT_ECHO_PIN 25

#define WALL_FRONT_TRIG_PIN 26 // חיישני צד ליישור - חיישן קדמי
#define WALL_FRONT_ECHO_PIN 27

#define WALL_REAR_TRIG_PIN 28 // חיישני צד ליישור - חיישן אחורי
#define WALL_REAR_ECHO_PIN 29

#define BACK_TRIG_PIN 30 // חיישן רוורס אחורי
#define BACK_ECHO_PIN 31 

// --- חיישני גבול אולטרסוניים למערכת ההרמה והצביעה ---
#define TOP_TRIG_PIN 34    // חיישן עליון - מזהה מתי הדיזה קרובה לתקרה
#define TOP_ECHO_PIN 35

#define BOTTOM_TRIG_PIN 36 // חיישן תחתון - מזהה מתי הדיזה קרובה לרצפה
#define BOTTOM_ECHO_PIN 37

// חיישן דיזה חדש - לזיהוי דלתות/חלונות למניעת ריסוס באוויר
#define NOZZLE_TRIG_PIN 38 
#define NOZZLE_ECHO_PIN 39

const float STOP_DISTANCE_CM = 5.0; // המרחק שבו נרצה שהעגלה תעצור מהרצפה/תקרה

// --- מכונת המצבים (State Machine) של הרובוט ---
// עושה סדר בלוגיקה במקום מלא if-ים מסובכים
#define STATE_IDLE                 0 // המתנה
#define STATE_DRIVE_ALONG_WALL     1 // נסיעה רגילה וריסוס
#define STATE_STOP_AT_CORNER       2 // הגענו לפינה - עוצרים
#define STATE_PAINT_STRIP_DOWN     3 // שלב הורדת הדיזה למטה
#define STATE_PREPARE_TURN         4 // חישוב לאיזה כיוון לפנות
#define STATE_TURNING_90           5 // פנייה במקום לפי גירוסקופ
#define STATE_ALIGN_PHASE_1        6 // יישור לקיר - גלגל רחוק מתקרב
#define STATE_ALIGN_PHASE_2        7 // יישור לקיר - גלגל קרוב מתקן
#define STATE_REVERSE_TO_START     8 // נסיעה אחורה כדי להתחיל סבב חדש
#define STATE_FINISH_CYCLE         9 

int currentState = STATE_IDLE; // מתחילים במצב עמידה

#define WALL_ON_LEFT  0
#define WALL_ON_RIGHT 1
int currentWallSide = WALL_ON_LEFT; // משתנה ששומר באיזה צד הקיר שלנו כרגע

// --- מהירויות עבודה ---
#define FORWARD  HIGH
#define BACKWARD LOW

// הגדרות PWM: ערכים בין 0 ל-255. 
// הערה: לא לשים 255 בנסיעה רגילה כי זה מהיר מדי והגלגלים מחליקים. 200 זה מעולה.
#define PWM_FAST    200  // מהירות שיוט לאורך הקיר
#define PWM_TURN    150  // מהירות פנייה (עדיף לאט כדי שהגירוסקופ לא יפספס)
#define PWM_ALIGN   100  // מהירות איטית ליישור סופי ומדויק
#define PWM_REVERSE 150  // רוורס
#define PWM_STOP    0    

// טיימרים כדי לא להשתמש ב-delay שתוקע את הקוד
unsigned long now_time, prev_sensor_time;

// משתנים לשמירת המרחקים מהחיישנים
float frontDist = 999.0;
float wallFrontDist = 999.0;
float wallRearDist = 999.0;
float backDist = 999.0;
float topDist = 999.0;
float bottomDist = 999.0;
float nozzleDist = 999.0; // משתנה לחיישן הדיזה

// מונה האנקודר - חייב להיות volatile כי אנחנו משנים אותו בתוך פונקציית פסיקה (ISR)!
volatile long liftPosition = 0; 
// const long TARGET_LIFT_BOTTOM = -5000; // כבר אין צורך, עברנו לחיישן אולטרסוני!

// הגדרות גירוסקופ MPU6050
const int MPU_ADDR = 0x68; // כתובת I2C (מתחבר לפינים 20,21 במגה)
float currentYaw = 0.0;    // זווית נוכחית בציר Z (סבסוב)
float targetYaw = 0.0;     // לאיזו זווית אנחנו רוצים להגיע בפנייה
unsigned long lastGyroTime = 0;

void setup() {
  Serial.begin(115200);

  // הגדרת כל פיני המנועים כיציאות
  pinMode(LEFT_ENA, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  pinMode(DC_LIFT_ENA, OUTPUT);
  pinMode(DC_LIFT_IN1, OUTPUT);
  pinMode(DC_LIFT_IN2, OUTPUT);
  
  pinMode(VALVE_PIN, OUTPUT);
  
  // מוודאים בטיחות: שהכל כבוי ברגע שמדליקים את החשמל
  setLeftMotor(FORWARD, PWM_STOP);
  setRightMotor(FORWARD, PWM_STOP);
  digitalWrite(DC_LIFT_IN1, LOW);
  digitalWrite(DC_LIFT_IN2, LOW);
  analogWrite(DC_LIFT_ENA, 0);
  digitalWrite(VALVE_PIN, VALVE_CLOSED); // ברז צבע סגור!

  // הגדרת פינים של חיישני נסיעה
  pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(WALL_FRONT_TRIG_PIN, OUTPUT); pinMode(WALL_FRONT_ECHO_PIN, INPUT);
  pinMode(WALL_REAR_TRIG_PIN, OUTPUT); pinMode(WALL_REAR_ECHO_PIN, INPUT);
  pinMode(BACK_TRIG_PIN, OUTPUT); pinMode(BACK_ECHO_PIN, INPUT);
  
  // הגדרת פינים של חיישני גבול ההרמה והדיזה
  pinMode(TOP_TRIG_PIN, OUTPUT); pinMode(TOP_ECHO_PIN, INPUT);
  pinMode(BOTTOM_TRIG_PIN, OUTPUT); pinMode(BOTTOM_ECHO_PIN, INPUT);
  pinMode(NOZZLE_TRIG_PIN, OUTPUT); pinMode(NOZZLE_ECHO_PIN, INPUT); // חיישן הדיזה החדש

  // הגדרת אנקודר וחיבור פסיקת חומרה (Interrupt) - מופעל בירידת מתח מ-1 ל-0
  pinMode(ENCODER_CLK_A, INPUT_PULLUP);
  pinMode(ENCODER_DT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_A), updateEncoder, FALLING);

  // הפעלת הגירוסקופ (כתיבת 0 לאוגר 6B מוציאה אותו ממצב שינה)
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  Serial.println("System Initialized! Starting in 2 seconds...");
  delay(2000); // נותן לרובוט שנייה להתייצב לפני שמתחילים
  
  // תחילת העבודה
  currentState = STATE_DRIVE_ALONG_WALL; 
  prev_sensor_time = millis();
  lastGyroTime = millis();
}

void loop() {
  now_time = millis();

  // דוגמים את כל החיישנים רק פעם ב-50 מילישניות (לא מהר מדי כדי לא להציף את הארדואינו)
  if (now_time - prev_sensor_time >= 50) {
    prev_sensor_time = now_time;
    frontDist = getDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    wallFrontDist = getDistance(WALL_FRONT_TRIG_PIN, WALL_FRONT_ECHO_PIN);
    wallRearDist = getDistance(WALL_REAR_TRIG_PIN, WALL_REAR_ECHO_PIN);
    backDist = getDistance(BACK_TRIG_PIN, BACK_ECHO_PIN);
    
    // דגימת חיישני הגבול העליון והתחתון, וחיישן הדיזה
    topDist = getDistance(TOP_TRIG_PIN, TOP_ECHO_PIN);
    bottomDist = getDistance(BOTTOM_TRIG_PIN, BOTTOM_ECHO_PIN);
    nozzleDist = getDistance(NOZZLE_TRIG_PIN, NOZZLE_ECHO_PIN); // מתעדכן ברציפות
    
    updateGyroYaw(); // מעדכן איפה אנחנו במרחב
  }

  // --- ביצוע הפעולות לפי המצב הנוכחי ---
  switch (currentState) {
// ========================================================   
    case STATE_IDLE:
      setLeftMotor(FORWARD, PWM_STOP);
      setRightMotor(FORWARD, PWM_STOP);
      break;
// =========================================================
    case STATE_DRIVE_ALONG_WALL:
      // נסיעה קדימה לאורך הקיר הקיים
      setLeftMotor(FORWARD, PWM_FAST);
      setRightMotor(FORWARD, PWM_FAST);
      
      // אם זיהינו קיר ממול (קרוב מ-20 ס"מ) - עוצרים כדי לצבוע את הפינה
      if (frontDist > 0 && frontDist <= 20.0) {
        Serial.println("Found a corner. Stopping.");
        currentState = STATE_STOP_AT_CORNER;
      }
      break;
// ==============================================
    case STATE_STOP_AT_CORNER:
      // קודם כל עוצרים את מנועי הנסיעה
      setLeftMotor(FORWARD, PWM_STOP);
      setRightMotor(FORWARD, PWM_STOP);
      delay(500); // השהייה קטנה לייצוב הרעידות
      
      // הערה לעצמי: אפשר לאפס פה את האנקודר ליתר ביטחון, למרות שעברנו לחיישן גבול
      noInterrupts();
      liftPosition = 0; 
      interrupts();
      
      // מפעילים את מנוע ההרמה כלפי מטה (לכיוון חיישן ה-BOTTOM)
      digitalWrite(DC_LIFT_IN1, HIGH);
      digitalWrite(DC_LIFT_IN2, LOW);
      analogWrite(DC_LIFT_ENA, 180); // עוצמה סבירה
      
      // לא פותחים אוטומטית, ההחלטה תתבצע בסטייט הבא לפי חיישן הדיזה
      currentState = STATE_PAINT_STRIP_DOWN;
      break;
// ===================================================
    case STATE_PAINT_STRIP_DOWN:
      // כל עוד המרחק מהרצפה גדול מ-15 ס"מ (או שיש שגיאת קריאה 0), ממשיכים
      if (bottomDist > STOP_DISTANCE_CM || bottomDist <= 0) {
         // עובד ברקע - המנוע מוריד את העגלה
         
         // ---> תוספת חדשה: בקרת זיהוי דלתות / חלונות בזמן אמת <---
         if (nozzleDist > 20.0 || nozzleDist <= 0) {
             // זיהינו מרחק חריג (דלת, חלון, חלל ריק) - עוצרים צביעה מיד!
             digitalWrite(VALVE_PIN, VALVE_CLOSED);
         } else {
             // אנחנו מול קיר במרחק תקין - אפשר לרסס צבע
             digitalWrite(VALVE_PIN, VALVE_OPEN);
         }
         
      } else {
        // הגענו למטה! החיישן זיהה שאנחנו קרובים לרצפה
        Serial.println("Floor Reached (15cm). Valve OFF.");
        digitalWrite(VALVE_PIN, VALVE_CLOSED); // סוגרים את ברז הצבע!
        
        // עוצרים את מנוע הדיזה
        digitalWrite(DC_LIFT_IN1, LOW);
        digitalWrite(DC_LIFT_IN2, LOW);
        analogWrite(DC_LIFT_ENA, 0);
        delay(500); 
        
        currentState = STATE_PREPARE_TURN;
      }
      break;
// ==========================================================================
    case STATE_PREPARE_TURN:
      // מחשבים את זווית היעד לפנייה. 
      // אם הקיר בצד שמאל, נרצה לפנות ימינה (מינוס 90) ולהיפך.
      if (currentWallSide == WALL_ON_LEFT) {
        targetYaw = currentYaw - 90.0; 
      } else {
        targetYaw = currentYaw + 90.0; 
      }
      currentState = STATE_TURNING_90;
      break;
// ==============================================================================
    case STATE_TURNING_90:
      // מבצעים פניית טנק (Pivot) - צד אחד נוסע אחורה, השני עוצר או נוסע קדימה
      if (currentWallSide == WALL_ON_LEFT) {
        setRightMotor(BACKWARD, PWM_TURN);
        setLeftMotor(FORWARD, PWM_STOP);
      } else {
        setLeftMotor(BACKWARD, PWM_TURN);
        setRightMotor(FORWARD, PWM_STOP);
      }
      
      // בודקים אם הגענו לזווית היעד (עם טווח טעות של 2 מעלות כדי שלא ירעד)
      if (abs(currentYaw - targetYaw) <= 2.0) { 
        Serial.println("90deg Turn Done.");
        currentState = STATE_ALIGN_PHASE_1;
      }
      break;
// ===========================================================================
    case STATE_ALIGN_PHASE_1:
      // שלב ראשון ביישור: מקדמים רק את הגלגל הרחוק עד שהוא מזהה קיר
      if (currentWallSide == WALL_ON_LEFT) {
        setRightMotor(FORWARD, PWM_ALIGN); 
        setLeftMotor(FORWARD, PWM_STOP);   
      } else {
        setLeftMotor(FORWARD, PWM_ALIGN);  
        setRightMotor(FORWARD, PWM_STOP);  
      }

      if (wallFrontDist > 0 && wallFrontDist <= 20.0) {
        Serial.println("Align Phase 1 Done.");
        setLeftMotor(FORWARD, PWM_STOP);
        setRightMotor(FORWARD, PWM_STOP);
        currentState = STATE_ALIGN_PHASE_2;
      }
      break;
// =========================================================================
    case STATE_ALIGN_PHASE_2:
      // שלב שני: מקדמים את הגלגל הקרוב עד ששני החיישנים מקבילים לקיר
      if (currentWallSide == WALL_ON_LEFT) {
        setLeftMotor(FORWARD, PWM_ALIGN); 
        setRightMotor(FORWARD, PWM_STOP);
      } else {
        setRightMotor(FORWARD, PWM_ALIGN); 
        setLeftMotor(FORWARD, PWM_STOP);
      }

      // בודקים יישור במרחק 20 ס"מ (פלוס מינוס 1 ס"מ סטייה מותרת)
      bool frontAligned = (wallFrontDist >= 19.0 && wallFrontDist <= 21.0);
      bool rearAligned  = (wallRearDist >= 19.0 && wallRearDist <= 21.0);

      if (frontAligned && rearAligned) {
         Serial.println("Robot is parallel to the wall!");
         setLeftMotor(FORWARD, PWM_STOP);
         setRightMotor(FORWARD, PWM_STOP);
         currentState = STATE_REVERSE_TO_START;
      }
      break;
//=============================================================
    case STATE_REVERSE_TO_START:
      // נוסעים אחורה קצת כדי להתמקם טוב לתחילת הפס הבא
      setLeftMotor(BACKWARD, PWM_REVERSE);
      setRightMotor(BACKWARD, PWM_REVERSE);

      // עוצרים כשאנחנו 5 ס"מ מהקיר מאחורינו
      if (backDist > 0 && backDist <= 5.0) {
        Serial.println("Ready for next wall.");
        setLeftMotor(FORWARD, PWM_STOP);
        setRightMotor(FORWARD, PWM_STOP);
        currentState = STATE_FINISH_CYCLE;
      }
      break;
// =============================================================================
    case STATE_FINISH_CYCLE:
      delay(1000); // קצת מנוחה למערכות 
      currentState = STATE_DRIVE_ALONG_WALL; // מתחילים הכל מחדש לקיר הבא!
      break;
  }
}

// ==========================================
// פונקציות עזר - חוסך המון שורות קוד חוזרות
// ==========================================
//-----------------------------------------------------
// פונקציה חכמה לשליטה במנוע שמאל - מקבלת כיוון ומהירות ומפעילה את הדרייבר
void setLeftMotor(int dir, int speed) {
  if (dir == FORWARD) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
  }
  analogWrite(LEFT_ENA, speed);
}
//----------------------------------------------------
// פונקציה חכמה לשליטה במנוע ימין
void setRightMotor(int dir, int speed) {
  if (dir == FORWARD) {
    digitalWrite(RIGHT_IN3, HIGH);
    digitalWrite(RIGHT_IN4, LOW);
  } else {
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, HIGH);
  }
  analogWrite(RIGHT_ENB, speed);
}
//-------------------------------------------------------
// פונקציה לקריאת מרחק מחיישן אולטרסוני
float getDistance(int trigPin, int echoPin) {
  // שולחים פולס של 10 מיקרושניות כדי לעורר את החיישן
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // קוראים את הזמן שלוקח להד לחזור.
  // שמתי פה טיימאאוט של 10,000 מיקרושניות כדי שאם אין קיר הקוד לא ייתקע וימתין לנצח.
  long duration = pulseIn(echoPin, HIGH, 10000); 
  if (duration == 0) return 999.0; // מחזיר מספר ענק אם לא זוהה כלום
  
  // מהירות הקול היא 340 מ/ש, עושים המרה לסנטימטרים ומחלקים ב-2 (הלוך חזור)
  return duration * 0.034 / 2;
}
//------------------------------------------------------------
// קריאת נתונים מהגירוסקופ (ציר הסבסוב בלבד)
void updateGyroYaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  
  int16_t gz = (Wire.read() << 8) | Wire.read();
  
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0; // חישוב דלתא זמן בשניות
  lastGyroTime = now;
  
  float gyroZRate = gz / 131.0; // המרה למעלות לשנייה לפי הדף נתונים של הרכיב
  
  // מתעלמים מתנועות ממש קטנות כדי שלא יצבור "דריפט" (רעש) כשהרובוט עומד
  if (abs(gyroZRate) > 1.0) {
    currentYaw += gyroZRate * dt; // מוסיפים את שינוי הזווית למשתנה הכולל
  }
}
//-------------------------------------------------------------------------
// פונקציית פסיקה (ISR) לאנקודר - רצה ברקע אוטומטית כשהמנוע הרמה זז
void updateEncoder() {
  if (digitalRead(ENCODER_DT_B) == HIGH) {
    liftPosition++; // המנוע יורד/עולה
  } else {
    liftPosition--; // המנוע מסתובב בכיוון ההפוך
  }
}

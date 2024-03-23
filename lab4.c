/**********************************************************************************************************************
Course: ROBT1270 - C Programming

Program: Lab4: SCARA robot basic control

Details: Control the SCARA robot using various commands.  Draw lines and curves with various colors and line
         thicknesses. The line or curve will only be drawn if it can be drawn in its entirety using
         the left arm or right arm without switching arms at any point.  Components of the C program include
         variables, formatted console output, error checked user input, branches, loops, functions,
         bitwise operations, structures.

Author(s): Tyler Gravenor : A0 and Calvin Mwin-Era : A01316868

Declaration: We, Tyler Gravenor and Calvin Mwin-Era, declare that the following program was written by us.

Date Created: March 06 2024

**********************************************************************************************************************/
#undef __cplusplus

//#define TEST

//-------------------------- Standard library prototypes --------------------------------------------------------------
#include <stdlib.h>     // standard functions and constant
#include <stdio.h>      // i/o functions
#include <math.h>       // math functions
#include <string.h>     // string functions
#include <ctype.h>      // character functions
#include <stdbool.h>    // bool definitions
#include <stdarg.h>     // for variable argument functions
#include <locale.h>     // for printing wide characters
#include "constants.h"  // program constants and structure definitions

#define dsprintf dsprintf // make all dsprintf calls use dsprintf

//-------------------------------- Robot Definitions and Function Prototypes ------------------------------------------
int initializeRobot();              // creates a TCP/IP connection between this program and the robot.
int sendRobotCommand(const char *); // sends a command remotely to the SCARA robot
void closeRobot();                  // closes the TCP/IP connection to the robot
SCARA_STATE getRobotState();        // gets the current state of the SCARA robot

//----------------------------- Given Function Prototypes -------------------------------------------------------------
void waitForEnterKey();          // waits for the Enter key to be pressed
void endProgram();               // ends the program from anywhere in the code
double degToRad(double);         // returns angle in radians from input angle in degrees
double radToDeg(double);         // returns angle in degrees from input angle in radians
double mapAngle(double angRad);  // make sure inverseKinematic angled are mapped in range robot understands
bool doAgain();                  // asks user if they want to draw another shape
int getShapeChoice();            // gets the shape choice to draw from the user (LINE/ARC/BEZIER)
void getInputString(char *strInput);   // gets user input from the keyboard (instead of scanf_s!)
int dsprintf(char *fmt, ...);          // mimics dsprintf but also prints to a log file
void printRepeatedChar(wchar_t ch, int numRepeats);               // prints a character repeatedly
void printTableHeader(int tableWidth, const char *strTableTitle); // prints table header with centered title
void printTableHBorder(wchar_t chL, wchar_t chR, int tableWidth); // prints table horizontal border 
// prints joint angle and reachability data into table for a given [x,y] coordinate.  
void printPointData(size_t iPt, size_t NP, POINT2D pos, JOINT_ANGLES leftArm, JOINT_ANGLES rightArm, int reachState);

//----------------------------- Manditory Function protoypes ----------------------------------------------------------
void drawShape(int shape); // Draws a line, arc, or quadratic bezier based on users choice (YOU NEED TO COMPLETE THIS!)
// inverseKinematics:      Gets joint angles from tool position
int inverseKinematics(INVERSE_SOLUTION *, TOOL_POSITION);
// setPenColor:            Sends command to the robot to set the robot pen color
// setPenPos:              Sends command to the robot to set the robot pen position
// setMotorSpeed:          Sends command to the robot to set the robot motor speed
// rotateRobotJoints:      Sends a ROTATE_JOINT command to the robot to move the arms
// drawLine:               Draws a line based on user input data
void drawLine();
// drawArc:                Draws an arc based on user input data
// drawQuadraticBezier:    Draws a quadratic bezier curve based on user input data

// getLineData:            Gets the line endpoints and number of points from the user
LINE_DATA getLineData(LINE_DATA);
// getArcData:             Gets the arc center, radius, and start and end angle from the user
ARC_DATA getArcData(ARC_DATA);
// getQuadraticBezierData: Gets the bezier control points and number of points from the user
QUADRATIC_BEZIER_DATA getQuadraticBeizerData(QUADRATIC_BEZIER_DATA);
// getNumPoints:           Gets the number of points used to draw a line, arc, or quadratic bezier curve
size_t getNumPoints();
// getTraceAttributes:     Gets the trace color and thickness use to draw a line, arc, or quadratic bezier curve
TRACE_ATTRIBUTES getTraceAttributes();
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  C program to draw straight lines with the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   int iShape = -1;  // index of shape to draw (LINE/ARC/BEZIER)

   system("chcp 65001");            // set the code page to 65001 (the UTF-8 character set)
   setlocale(LC_ALL, ".UTF-8");     // ensure the printing follows the character set

   if(!initializeRobot()) exit(0);  // connect to the robot

   do  // draw shapes until user terminates
   {
      iShape = getShapeChoice(); // get users choice of shape to draw
      drawShape(iShape);         // draw requested shape
   }
   while(doAgain());

   sendRobotCommand("HOME\n");
   endProgram();
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: 
// ARGUMENTS:   
// RETURN VALUE:
void drawLine()
{
   //variables
   LINE_DATA line = {0};
   size_t iPoint = 0;
   TOOL_POSITION tp = {0};
   INVERSE_SOLUTION isol = {0};
   double t = NAN, n = 1; //hard coded n - value
   int reachState = 0;       // reachability state for both arms for given tooltip point x,y


   //function call to get the user input data to draw the line.
   getLineData(line);

   //Table header.
   int numChars = 0;
   printTableHBorder(VL, VL, TABLE_WIDTH);  
   dsprintf("\nLINE: P0 = [%+lg, %+lg],  P1 = [%+lg, %+lg], NP = %zu\n",
      line.P0.x, line.P0.y, line.P1.x, line.P1.y, line.NP);
   printTableHBorder(VL, VL, TABLE_WIDTH);



   //Calculating points between line end points.
   for(iPoint = 0; iPoint < line.NP; iPoint++)
   {
      //Exponential for parametric line distribution
      t = (double)iPoint / ((double)line.NP - (double)1);
      t = pow(t, n);
      tp.x = (((double)1 - t) * line.P0.x) + t * line.P1.x;
      tp.y = (((double)1 - t) * line.P0.y) + t * line.P1.y;


      reachState = inverseKinematics(&isol, tp);
      printPointData(iPoint, line.NP, tp, isol.leftArm, isol.rightArm, reachState);
      
   }
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: 
// ARGUMENTS:   
// RETURN VALUE:
TRACE_ATTRIBUTES getTraceAttributes()
{
   TRACE_ATTRIBUTES trace = {0};
   size_t n = 0;
   char penData;
   int iret = 0;

   //put this in while loop.
   dsprintf("Choose the letter corresponding to the pen colour you want: ");
   dsprintf("A=Aqua\tB=Black\tG=Green\tH=HotPink\tO=Orange\nR=Red\tW=White\tN=Navy\tY=Yellow\tP=Purple\n");
  
   dsprintf("Choice: \n");
   sscanf("%c", &penData); //do errror checking

   switch(penData)
   {
      case 'A':
         trace.penColor = AQUA;
         break;
      case 'B':
         trace.penColor = BLACK;
         break;
      case 'G':
         trace.penColor = GREEN;
         break;
      case 'H':
         trace.penColor = HOTPINK;
         break;
      case 'O':
         trace.penColor = ORANGE;
         break;
      case 'R':
         trace.penColor = RED;
         break;
      case 'W':
         trace.penColor = WHITE;
         break;
      case 'N':
         trace.penColor = NAVY;
         break;
      case 'Y':
         trace.penColor = YELLOW;
         break;
      case 'P':
         trace.penColor = PURPLE;
         break;
      default:
         dsprintf("Please enter a valid point\n");
         break;
   }

}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: gets the start, mid and end points required to draw the quadratic beizer curve - DONE!
// ARGUMENTS:   
// RETURN VALUE:
QUADRATIC_BEZIER_DATA getQuadraticBeizerData(QUADRATIC_BEZIER_DATA quadBez)
{
   int iret = -1;                      // scanf_s return value
   char strInput[MAX_BUFF];           // get user input as a string
   char strGarbage[2] = {0};         // for storing the first garbage character

   quadBez.NP = getNumPoints();     //Function call to get number of points.
   quadBez.P0.x = 0, quadBez.P0.y = 0, quadBez.P1.x = 0, quadBez.P1.y = 0, quadBez.P2.x = 0, quadBez.P2.y = 0;


   // get 6 double values
   while(true)
   {
      dsprintf("Enter p0x, p0y, p1x, p1y, p2x, p2y (comma separated): ");
      getInputString(strInput); // store the users input
      strInput[strlen(strInput) - 1] = '\0'; // remove the '\n' that is stored in strInput

      // strGarbage will store the 1st garbage character. The 2 is the size (in characters) of the strGarbage array
      iret = sscanf_s(strInput, "%lf , %lf , %lf, %lf, %lf, %lf%1s", 
         &quadBez.P0.x, &quadBez.P0.y, &quadBez.P1.x, &quadBez.P1.y, &quadBez.P2.x, &quadBez.P2.y, strGarbage, 2);

      if(iret == 6) break; // got the 3 required values without any trailing garbage
      if(iret == 7) // strGarbage collected some trailing garbage
         dsprintf("I got six values (%lg, %lg, %lg, %lg, %lg, %lg) but input has trailing garbage.\n",
            quadBez.P0.x, quadBez.P0.y, quadBez.P1.x, quadBez.P1.y, quadBez.P2.x, quadBez.P2.y);
      else if(iret == 5)
         dsprintf("I got five values (%lg, %lg, %lg, %lg, %lg). ", quadBez.P0.x, quadBez.P0.y, quadBez.P1.x, quadBez.P1.y, quadBez.P2.x);
      else if(iret == 4)
         dsprintf("I got four values (%lg, %lg, %lg, %lg). ", quadBez.P0.x, quadBez.P0.y, quadBez.P1.x, quadBez.P1.y);
      else if(iret == 3)
         dsprintf("I got three values (%lg, %lg, %lg). ", quadBez.P0.x, quadBez.P0.y, quadBez.P1.x);
      else if(iret == 2)
         dsprintf("I got two values (%lg, %lg). ", quadBez.P0.x, quadBez.P0.y);
      else if(iret == 1)
         dsprintf("I got one value (%lg). ", quadBez.P0.x);
      else if(iret == 0)
         dsprintf("I didn't get anything usable because you have leading garbage\n");
      else // iret == -1
         dsprintf("Please enter _something_!\n");
      if(iret > 0 && iret < 7) dsprintf("There is missing data and possibly missing commas and/or trailing garbage.\n");
   }
   return quadBez;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: gets the start and end angles; center point and radius required to draw the arc. - done!
// ARGUMENTS:   arc
// RETURN VALUE: arc
ARC_DATA getArcData(ARC_DATA arc)
{
   int iret = -1;                       // scanf_s return value
   char strInput[MAX_BUFF];            // get user input as a string
   char strGarbage[2] = {0};          // for storing the first garbage character
        
   //Initializing structure contents.
   arc.NP = getNumPoints();       //gets number of points from return value of function call.
   arc.thetaStartDeg = 0.0, arc.thetaEndDeg = 0.0;
   arc.pc.x = 0.0, arc.pc.y = 0.0;
   arc.r = 0.0;

   // get 5 integer values
   while(true)
   {
      dsprintf("Enter theta start, theta end, center x, center y and radius (comma separated): ");
      getInputString(strInput);                    // store the users input
      strInput[strlen(strInput) - 1] = '\0';       // remove the '\n' that is stored in strInput
      // strGarbage will store the 1st garbage character. The 2 is the size (in characters) of the strGarbage array

      iret = sscanf_s(strInput, "%lf , %lf , %lf, %lf, %lf%1s",
         &arc.thetaStartDeg, &arc.thetaEndDeg, &arc.pc.x, &arc.pc.y, &arc.r, strGarbage, 2);
      if(iret == 5) break; // got the 5 required values without any trailing garbage
      if(iret == 6) // strGarbage collected some trailing garbage
         dsprintf("I got five values (%lg, %lg, %lg, %lg, %lg) but input has trailing garbage.\n",
            arc.thetaStartDeg, arc.thetaEndDeg, arc.pc.x, arc.pc.y, arc.r);
      else if(iret == 4)
         dsprintf("I got four values (%lg, %lg, %lg, %lg). ", arc.thetaStartDeg, arc.thetaEndDeg, arc.pc.x, arc.pc.y);
      else if(iret == 3)
         dsprintf("I got three values (%lg, %lg, %lg). ", arc.thetaStartDeg, arc.thetaEndDeg, arc.pc.x);
      else if(iret == 2)
         dsprintf("I got two values (%lg, %lg). ", arc.thetaStartDeg, arc.thetaEndDeg);
      else if(iret == 1)
         dsprintf("I got one value (%lg). ", arc.thetaStartDeg);
      else if(iret == 0)
         dsprintf("I didn't get anything usable because you have leading garbage\n");
      else // iret == -1
         dsprintf("Please enter _something_!\n");
      if(iret > 0 && iret < 6) dsprintf("There is missing data and possibly missing commas and/or trailing garbage.\n");
   }
   return arc;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: gets the start and end points for drawing a line  - done!
// ARGUMENTS:   line
// RETURN VALUE: line
LINE_DATA getLineData(LINE_DATA line)
{
   int iret = -1;                    // scanf_s return value
   char strInput[MAX_BUFF];         // get user input as a string
   char strGarbage[2] = {0};       // for storing the first garbage character
   
   line.P0.x = 0.0, line.P0.y = 0.0;    //test variable for start point.
   line.P1.x = 0.0, line.P1.y = 0.0;    //test variable for end point.
   line.NP = getNumPoints();        //function call for number of points for given shape.


   // get 4 double values for line start and end points.
   while(true)
   {
      dsprintf("Enter line end points p0x, p0y, p1x, p1y (comma separated): ");
      getInputString(strInput);                       // store the users input
      strInput[strlen(strInput) - 1] = '\0';          // remove the '\n' that is stored in strInput
      // strGarbage will store the 1st garbage character. The 2 is the size (in characters) of the strGarbage array
      iret = sscanf_s(strInput, "%lf, %lf, %lf, %lf%1s", 
         &line.P0.x, &line.P0.y, &line.P1.x, &line.P1.y, strGarbage, (unsigned)sizeof(strGarbage));

      if(iret == 4) break;          // got the 4 required values without any trailing garbage
      if(iret == 5)                // strGarbage collected some trailing garbage
         dsprintf("I got four values (%lg, %lg, %lg, %lg) but input has trailing garbage.\n", 
            line.P0.x, line.P0.y, line.P1.x, line.P1.y);
      else if(iret == 3)
         dsprintf("I got three values (%lg, %lg, %lg). ", line.P0.x, line.P0.y, line.P1.x);
      else if(iret == 2)
         dsprintf("I got two values (%lg, %lg). ", line.P0.x, line.P0.y);
      else if(iret == 1)
         dsprintf("I got one value (%lg). ", line.P0.x);
      else if(iret == 0)
         dsprintf("I didn't get anything usable because you have leading garbage\n");
      else // iret == -1
         dsprintf("Please enter _something_!\n");
      if(iret > 0 && iret < 5) dsprintf("There is missing data and possibly missing commas and/or trailing garbage.\n");
   }
   return line;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Gets the number of points for drawing the line or shape. - DONE!
// ARGUMENTS:    None
// RETURN VALUE: numPoints
size_t getNumPoints()
{
   int iret = -1;                     // scanf_s return value
   char strInput[MAX_BUFF];          // get user input as a string
   char strGarbage[2] = {0};        // for storing the first garbage character
   size_t numPoints = 0;           // test data variables

   // get unsigned integer values
   while(true)
   {
      dsprintf("Enter the number of points: ");
      getInputString(strInput);                     // store the users input
      strInput[strlen(strInput) - 1] = '\0';       // remove the '\n' that is stored in strInput
      // strGarbage will store the 1st garbage character. The 2 is the size (in characters) of the strGarbage array
      iret = sscanf_s(strInput, "%zu%1s", &numPoints, strGarbage, 2);

      if(iret == 1) break;       // got the  required value without any trailing garbage
      if(iret == 2)             // strGarbage collected some trailing garbage
         dsprintf("I got one value (%zu) but input has trailing garbage.\n", numPoints);
      else if(iret == 0)
         dsprintf("I didn't get anything usable because you have leading garbage\n");
      else // iret == -1
         dsprintf("Please enter _something_!\n");
      if(iret > 0 && iret < 2) dsprintf("There is missing data and possibly missing commas and/or trailing garbage.\n");
   }
   return numPoints;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: //Not done
// ARGUMENTS:    
// RETURN VALUE:
int inverseKinematics(INVERSE_SOLUTION *isol, TOOL_POSITION tp)
{
   //tp values will have to come from an outside source
   isol->leftArm.theta1Deg = 0;
   isol->leftArm.theta2Deg = 0;
   isol->rightArm.theta1Deg = 0;
   isol->rightArm.theta2Deg = 0;
   isol->bLeftCanReach = true;
   isol->bRightCanReach = true;

   //Initializing variables for calculations in radians.
   double theta1Lrad = 0, theta2Lrad = 0, theta1Rrad = 0, theta2Rrad = 0;

   double L = -1, alpha = 0, beta = 0;
   int prState = 0; //set to zero so that its all zeroed out for the bit pattern

   //Calculating L to determine robot reach.
   L = sqrt(pow(tp.x, 2) + pow(tp.y, 2));
   beta = atan2(tp.y, tp.x);
   alpha = acos((pow(L2, 2) - pow(L, 2) - pow(L1, 2)) / (-2 * L * L1));

   //Checks to make sure L is within reach before computing angles.
   if(L < LMIN)
   {
      prState = prState | L_EXCEEDS_MIN;
      return prState;
   }
   else if(L > LMAX)
   {
      prState = prState | L_EXCEEDS_MAX;
      return prState;
   }
   
   //for left arm
   theta1Lrad = beta + alpha;
   theta2Lrad = atan2((tp.y - (L1 * sin(theta1Lrad))), (tp.x - (L1 * cos(theta1Lrad)))) - theta1Lrad;
   //Using magAngle function to put thetas in range.
   mapAngle(theta1Lrad);
   mapAngle(theta2Lrad);

   isol->leftArm.theta1Deg = radToDeg(theta1Lrad);
   isol->leftArm.theta2Deg = radToDeg(theta2Lrad);

   //for right arm
   theta1Rrad = beta - alpha;
   theta2Rrad = atan2((tp.y - (L1 * sin(theta1Rrad))), (tp.x - (L1 * cos(theta1Rrad)))) - theta1Rrad;
   //Using magAngle function to put thetas in range.
   mapAngle(theta1Rrad);
   mapAngle(theta2Rrad);


   isol->rightArm.theta1Deg = radToDeg(theta1Rrad);
   isol->rightArm.theta2Deg = radToDeg(theta2Rrad);

   if(fabs(isol->leftArm.theta1Deg) > ABS_THETA1_DEG_MAX)
   {
      prState = prState | THETA1L_EXCEEDS_MAX;
      return prState;
   }
   if(fabs(isol->leftArm.theta2Deg) > ABS_THETA2_DEG_MAX)
   {
      prState = prState | THETA2L_EXCEEDS_MAX;
      return prState;
   }
   if(fabs(isol->rightArm.theta1Deg) > ABS_THETA1_DEG_MAX)
   {
      prState = prState | THETA1R_EXCEEDS_MAX;
      return prState;
   }
   if(fabs(isol->rightArm.theta2Deg) > ABS_THETA2_DEG_MAX)
   {
      prState = prState | THETA2R_EXCEEDS_MAX;
      return prState;
   }

}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user what shape they want to draw
// ARGUMENTS:    none
// RETURN VALUE: shape choice index (see enum SHAPES for values)
int getShapeChoice()
{
   int iret = -1;                // store scanf_s return value
   char ch = 0;                  // store users input
   char strInput[MAX_BUFF];      // stores the users input (as a string)

   while(true)  // ask until get valid data
   {
      dsprintf("What shape would you like to draw [L for Line, A for Arc, Q for Quadratic Bezier]: ");
      getInputString(strInput);
      iret = sscanf_s(strInput, "%c", &ch, 1);

      if(strlen(strInput) == 2)  // single character + '\n', check if valid character
      {
         ch = (char)toupper(ch);
         if(ch == 'L' || ch == 'A' || ch == 'Q')
         {
            if(ch == 'L')
               return LINE;
            else if(ch == 'A')
               return ARC;
            else
               return QUADRATIC_BEZIER;
         }
         else
         {
            dsprintf("Bad input! Please enter L or A or Q\n");
         }
      }
      else // multiple characters
      {
         dsprintf("please enter a single character.\n");
      }
   }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  draws a shape with a robot according to user specifications
// ARGUMENTS:    shape: index of the shape to draw (LINE/ARC/BEZIER)
// RETURN VALUE: none
void drawShape(int shape)
{

}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Asks the user if they want to draw another line
// ARGUMENTS:    none
// RETURN VALUE: true if they want to, false if not
bool doAgain()
{
   bool bDoAgain = false;        // users choice as a bool
   int iret = -1;                // store scanf_s return value   
   char ch = 0;                  // store users input
   char strInput[MAX_BUFF];   // get user input as a string

   while(true)  // ask until get valid data
   {
      dsprintf("\nDo you want to draw another shape [y/n]? ");
      getInputString(strInput);
      iret = sscanf_s(strInput, "%c", &ch, 1);

      if(strlen(strInput) == 2)  // single character + '\n', check if valid character
      {
         ch = (char)tolower(ch);    // in case caps lock is on

         if(ch == 'y' || ch == 'n')  // good data
         {
            bDoAgain = (ch == 'y' ? true : false);
            break;
         }
         else // invalid character
            dsprintf("please enter 'y' or 'n'\n");
      }
      else // multiple characters
      {
         dsprintf("please enter a single character.\n");
      }
   }

   return bDoAgain;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets user input as a string
// ARGUMENTS:    strInput:  string used to store the input
// RETURN VALUE: none
void getInputString(char *strInput)
{
   static char buff[MAX_BUFF]; // temp char array

   fgets(strInput, MAX_BUFF, stdin);      // get all user input character as a string (including the '\n'!)
   strcpy_s(buff, MAX_BUFF, strInput);    // copy the input string
   strcat_s(buff, MAX_BUFF, strHack);     // append the hack string onto the end of the user input string
   dsprintf(buff);                          // echo the user input to the file (hack prevents console echo)
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints a table header with centered title (includes top/bottom borders)
// ARGUMENTS:    tableWdith:     width of table included left/right borders
//               strTableTitle:  title string
// RETURN VALUE: none
void printTableHeader(int tableWidth, const char *strTableTitle)
{
   int numChars = -1;   // number of characters printed by dsprintf
   int titleWidth = -1; // number of characters in the table title
   int leftSpaces = -1; // spaces to left of title
   int exVL = snprintf(NULL, 0, "%lc", VL) - 1;  // extra bytes need to adjust dsprintf for VL character :(

   titleWidth = (int)strlen(strTableTitle);  // title width

   leftSpaces = (tableWidth - 2 - titleWidth) / 2;  // -2 for the left/right borders
   
   printTableHBorder(TL, TR, tableWidth); // top border

   // header title
   numChars = dsprintf("%lc%*c%s", VL, leftSpaces, ' ', strTableTitle) - exVL; // dsprintf returns more than 1 to print VL!
   dsprintf("%*lc\n", tableWidth - numChars + exVL, VL); // +exVL for right border

   printTableHBorder(CL, CR, tableWidth);  // mid border (excepts table data row below the header)
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints data for one point into table (may be multiple lines).  prints bottom border but not top
// ARGUMENTS:    iPt:      The zero-based point index of the line/arc/quadratic bezier
//               NP:          The number of points of the line/arc/quadratic bezier, including endpoints
//               pos:         The x,y coordinates of the point
//               leftArm:     Joint angles (in degrees) for the left arm solution
//               rightArm:    Joint angles (in degrees) for the right arm solution
//               reachState:  The combined reach state data (for contextual printing)
// RETURN VALUE: ????
void printPointData(size_t iPt, size_t NP, POINT2D pos, JOINT_ANGLES leftArm, JOINT_ANGLES rightArm, int reachState)
{
   int n = -1;                                                // number of characters returned by dsprintf
   wchar_t chL = 0, chR = 0;                                  // left/right border characters
   double L = sqrt(pos.x * pos.x + pos.y * pos.y);            // distance to pen tool from robot base
   int exVL = snprintf(NULL, 0, "%lc", VL) - 1;               // extra bytes needed to adjust dsprintf for VL character :(
   int exDeg = snprintf(NULL, 0, "%lc", DEGREE_SYMBOL) - 1;   // extra bytes for DEGREE_SYMBOL :(
   int exTheta = snprintf(NULL, 0, "%lc", THETA_SYMBOL) - 1;  // extra bytes for THETA_SYMBOL :(

   // coordinates and L (print all text up to right spaces)
   n = dsprintf("%lc%*cPOINT %02zu:   x = %+*.*lf,   y = %+*.*lf.   L = %+*.*lf", VL, LEFT_MARGIN, ' ', iPt + 1,
      FIELD_WIDTH, PRECISION, pos.x, FIELD_WIDTH, PRECISION, pos.y, FIELD_WIDTH, PRECISION, L) - exVL;
   dsprintf("%*lc\n", TABLE_WIDTH - n + exVL, VL); // right spaces and border

   if(reachState == L_EXCEEDS_MIN || reachState == L_EXCEEDS_MAX)  // unreachable because of L limits
   {
      n = dsprintf("%lc%*c", VL, LEFT_MARGIN, ' ') - exVL;  // left margin

      printRepeatedChar(ERROR_SYMBOL_LEFT, NUM_ERROR_SYMBOLS); // left error symbols
      n += NUM_ERROR_SYMBOLS;

      // error message
      if(reachState == L_EXCEEDS_MAX)
         n += dsprintf(" POINT IS OUTSIDE MAXIMUM REACH OF ROBOT (L_MAX = %.*lf) ", LIMIT_PRECISION, LMAX);
      else
         n += dsprintf(" POINT IS INSIDE MINIMUM REACH OF ROBOT (L_MIN = %.*lf) ", LIMIT_PRECISION, LMIN);

      printRepeatedChar(ERROR_SYMBOL_RIGHT, NUM_ERROR_SYMBOLS); // right error symbols
      n += NUM_ERROR_SYMBOLS;

      dsprintf("%*lc\n", TABLE_WIDTH - n + exVL, VL);  // right spaces and border
   }
   else // reachable in terms of L
   {
      //------------- LEFT ARM DATA -------------
      // left arm theta values
      n = dsprintf("%lc%*cLEFT ARM:  %lc1 = %+*.*lf%lc, %lc2 = %+*.*lf%lc.  ", VL, LEFT_MARGIN, ' ',
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, leftArm.theta1Deg, DEGREE_SYMBOL,
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, leftArm.theta2Deg, DEGREE_SYMBOL)
         - exVL - 2 * exTheta - 2 * exDeg;

      // left arm theta errors
      if(reachState & THETA1L_EXCEEDS_MAX || reachState & THETA2L_EXCEEDS_MAX)
      {
         if(reachState & THETA1L_EXCEEDS_MAX && reachState & THETA2L_EXCEEDS_MAX)
            n += dsprintf("%lc1 and %lc2 exceed max angle!", THETA_SYMBOL, THETA_SYMBOL) - 2 * exTheta;
         else if(reachState & THETA1L_EXCEEDS_MAX)
            n += dsprintf("%lc1 exceeds max angle!", THETA_SYMBOL) - exTheta;
         else
            n += dsprintf("%lc2 exceeds max angle!", THETA_SYMBOL) - exTheta;
      }
      dsprintf("%*lc\n", TABLE_WIDTH - n + exVL, VL);  // right spaces and border

      //------------- RIGHT ARM DATA -------------
      // right arm theta values
      n = dsprintf("%lc%*cRIGHT ARM: %lc1 = %+*.*lf%lc, %lc2 = %+*.*lf%lc.  ", VL, LEFT_MARGIN, ' ',
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, rightArm.theta1Deg, DEGREE_SYMBOL,
         THETA_SYMBOL, FIELD_WIDTH, PRECISION, rightArm.theta2Deg, DEGREE_SYMBOL)
         - exVL - 2 * exTheta - 2 * exDeg;

      // right arm theta errors
      if(reachState & THETA1R_EXCEEDS_MAX || reachState & THETA2R_EXCEEDS_MAX)
      {
         if(reachState & THETA1R_EXCEEDS_MAX && reachState & THETA2R_EXCEEDS_MAX)
            n += dsprintf("%lc1 and %lc2 exceed max angle!", THETA_SYMBOL, THETA_SYMBOL) - 2 * exTheta;
         else if(reachState & THETA1R_EXCEEDS_MAX)
            n += dsprintf("%lc1 exceeds max angle!", THETA_SYMBOL) - exTheta;
         else
            n += dsprintf("%lc2 exceeds max angle!", THETA_SYMBOL) - exTheta;
      }
      dsprintf("%*lc\n", TABLE_WIDTH - n + exVL, VL); // right spaces and border
   }

   // bottom border (use bottom left/right symbols if last point, othewize mid left/right symbols)
   chL = iPt == NP - 1 ? BL : CL;
   chR = iPt == NP - 1 ? BR : CR;
   printTableHBorder(chL, chR, TABLE_WIDTH);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a horiztonal border for a table
// ARGUMENTS:    chL/chR: left/right border characters
//               tableWidth: with of table including borders
// RETURN VALUE: none
void printTableHBorder(wchar_t chL, wchar_t chR, int tableWidth)
{
   dsprintf("%lc", chL);
   printRepeatedChar(HL, tableWidth - 2);
   dsprintf("%lc\n", chR);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Prints a character to the console repeatedly
// ARGUMENTS:    ch: character to repeat
//               numRepeats: number of repeats
// RETURN VALUE: none
void printRepeatedChar(wchar_t ch, int numRepeats)
{
   int i = -1; // loop counter

   for(i = 0; i < numRepeats; i++)
   {
      dsprintf("%lc", ch);
   }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle in degrees that follows the angle ranges 
//               defined in the robot (-180 <= theta <= +180)
// ARGUMENTS:    ang: the angle in radians 
// RETURN VALUE: the mapped angle in radians
double mapAngle(double theta)
{
   theta = fmod(theta, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if(theta > PI)
      theta -= 2.0 * PI;
   else if(theta < -PI)
      theta += 2.0 * PI;

   return radToDeg(theta);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad)
{
   return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits for user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey()
{
   char buff[MAX_BUFF];
   fgets(buff, MAX_BUFF, stdin);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Ends program from anywhere in code.
// ARGUMENTS:    none
// RETURN VALUE: none
void endProgram()
{
   dsprintf("\nPress ENTER to end the program...\n");
   waitForEnterKey();
   exit(0);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints to both a file and to the console
// ARGUMENTS:    f:  the file handle
//               fmt, ...: for variable number of parameters
// RETURN VALUE: the number of characters printed
int dsprintf(char *fmt, ...)
{
   static bool bIsFirst = true;  // to determine if first entry to function
   va_list args;                 // list of arguments to function
   int n1 = -1, n2 = -1;         // fix a glitch in number of characters printed:  file vs screen
   FILE *flog = NULL;            // for writing to a log file
   errno_t err = 0;              // detect error opening log file
   char fmt2[1024];              // hack!

   // if first time in, open in write mode to clear the previous file contents else open in append mode
   if(bIsFirst)
   {
      err = fopen_s(&flog, "log.txt", "w");
      bIsFirst = false;
   }
   else
   {
      err = fopen_s(&flog, "log.txt", "a");
   }

   // can't open!
   if(flog == NULL || err != 0)
   {
      dsprintf("Cannot open log.txt for writing!!\n");
      dsprintf("Please check that log.txt is not opened by another program.\n");
      dsprintf("Press ENTER to end this program");
      waitForEnterKey();
      exit(0);
   }

   // write to console
   if(strstr(fmt, strHack) == NULL)
   {
      va_start(args, fmt);
      n2 = vfprintf(stdout, fmt, args);
      va_end(args);
   }

   // write to log file
   strcpy_s(fmt2, 1024, fmt);
   if(strstr(fmt2, strHack) != NULL)
   {
      char *p = strstr(fmt2, strHack); // cut off the appended hack string
      if(p) *p = 0;
   }
   fmt = fmt2;  // must be const pointer!

   va_start(args, fmt);
   n1 = vfprintf(flog, fmt, args);
   va_end(args);

   fclose(flog); // close the file

   // fix glitch in number of characters returned
   if(n2 < n1) n1 = n2;
   return n1;
}


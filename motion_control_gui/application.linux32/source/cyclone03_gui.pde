//DV2/8/14  to be cleaned up. now runs with cyclone 02/03 firmware
/********************************************************
 * Arduino PID Tuning Front-End,  Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 *
 * This application is designed to interface with an
 * arduino running the PID Library.  From this Control
 * Panel you can observe & adjust PID performance in 
 * real time
 *
 * The ControlP5 library is required to run this sketch.
 * files and install instructions can be found at
 * http://www.sojamo.de/libraries/controlP5/
 * 
 ********************************************************/

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

/***********************************************
 * User spcification section
 **********************************************/
int windowWidth = 900;      // set the size of the 
int windowHeight = 600;     // form

float InScaleMin = -7500;       // set the Y-Axis Min
float InScaleMax = 7500;//1024;    // and Max for both
float OutScaleMin = -255;      // the top and 
float OutScaleMax = 255;    // bottom trends


int windowSpan = 30000;//300000;    // number of mS into the past you want to display
int refreshRate = 100;      // how often you want the graph to be reDrawn;

//float displayFactor = 1; //display Time as Milliseconds
float displayFactor = 1000; //display Time as Seconds
//float displayFactor = 60000; //display Time as Minutes

String outputFileName = ""; // if you'd like to output data to 
// a file, specify the path here

/***********************************************
 * end user spec
 **********************************************/

int nextRefresh;
int arrayLength = windowSpan / refreshRate+1;
int[] InputData = new int[arrayLength];     //we might not need them this big, but
int[] SetpointData = new int[arrayLength];  // this is worst case
int[] OutputData = new int[arrayLength];
int[] CurrentData = new int[arrayLength];
int[] WaveformDataE = new int[arrayLength];

float inputTop = 25;
float inputHeight = (windowHeight-70)*2/3;
float outputTop = inputHeight+50;
float outputHeight = (windowHeight-70)*1/3;

float ioLeft = 150, ioWidth = windowWidth-ioLeft-50;
float ioRight = ioLeft+ioWidth;
float pointWidth= (ioWidth)/float(arrayLength-1);

int vertCount = 10;

int nPoints = 0;

float Input, Setpoint, Output, Current, WaveformE;
float SP, oldSP;

boolean madeContact =false;
boolean justSent = true;

Serial myPort;

ControlP5 controlP5;
controlP5.Button AMButton, DRButton;
controlP5.Textlabel ConnectLabel, AMLabel, AMCurrent, InLabel, 
OutLabel, SPLabel, PLabel, 
ILabel, DLabel,DRLabel, DRCurrent, CurrentLabel;
controlP5.Textfield SPField, InField, OutField,
PField, IField, DField;//, CurrentField;

PrintWriter output;
PFont AxisFont, TitleFont; 

void setup()
{
  frameRate(30);
  size(windowWidth , windowHeight);

  println(Serial.list());                                           // * Initialize Serial
  myPort = new Serial(this, Serial.list()[0], 115200);                //   Communication with
  myPort.bufferUntil(10);                                           //   the Arduino

  controlP5 = new ControlP5(this);                                  // * Initialize the various
  SPField= controlP5.addTextfield("Setpoint",10,100,60,20);         //   Buttons, Labels, and
  InField = controlP5.addTextfield("Input",10,150,60,20);           //   Text Fields we'll be
  OutField = controlP5.addTextfield("Output",10,200,60,20);         //   using
  //CurrentField = controlP5.addTextfield("Current",10,220,60,20);
  PField = controlP5.addTextfield("Kp (Proportional)",10,275,60,20);          //
  IField = controlP5.addTextfield("Ki (Integral)",10,325,60,20);          //
  DField = controlP5.addTextfield("Kd (Derivative)",10,375,60,20);          //

  ConnectLabel = controlP5.addTextlabel("Conn","NoConn",80,20);

  AMButton = controlP5.addButton("Toggle_AM",0.0,10,50,60,20);      //
  AMLabel = controlP5.addTextlabel("AM","Manual",12,72);            //
  AMCurrent = controlP5.addTextlabel("AMCurrent","Manual",80,65);   //
  controlP5.addButton("Send_PID",0.0,10,475,60,20);         //

  controlP5.addButton("Send_SP",0.0,10,240,60,20);         //
  controlP5.addButton("Stop",0.0,110,320,40,20);

  
  SPLabel=controlP5.addTextlabel("SP","3",80,103);                  //
  InLabel=controlP5.addTextlabel("In","1",80,153);                  //
  OutLabel=controlP5.addTextlabel("Out","2",80,203);                //
  CurrentLabel=controlP5.addTextlabel("Current","7",80,213);     
  PLabel=controlP5.addTextlabel("P","4",80,278);                    //
  ILabel=controlP5.addTextlabel("I","5",80,328);                    //
  DLabel=controlP5.addTextlabel("D","6",80,378);                    //

  DRButton = controlP5.addButton("Toggle_DR",0.0,10,425,60,20);      //
  DRLabel = controlP5.addTextlabel("DR","Direct",12,447);            //
  DRCurrent = controlP5.addTextlabel("DRCurrent","Direct",80,440);   //

  controlP5.addSlider("SPslider")
       .setRange(InScaleMin,InScaleMax)
       .setValue(0)
       .setPosition(120, 100)
       .setNumberOfTickMarks(23)
       .setSize(15,200);

  AxisFont = loadFont("axis.vlw");
  TitleFont = loadFont("Titles.vlw");
 
  nextRefresh=millis();
  if (outputFileName!="") output = createWriter(outputFileName);
}
void draw()
{
  background(200);
  drawGraph();
  drawButtonArea();
  float SP = controlP5.getController("SPslider").getValue();
//  oldSP = float(SPField.getText());
  if(oldSP !=  SP) {
      //SPField.setValue( setpoint );  
      SPField.setText( str(SP) );
      println("new set point " + SP + " old " + oldSP);
      oldSP = SP; 
      Send_SP();
  }
}

void drawGraph()
{
  //draw Base, gridlines
  stroke(0);
  fill(230);
  rect(ioLeft, inputTop,ioWidth-1 , inputHeight);
  rect(ioLeft, outputTop, ioWidth-1, outputHeight);
  stroke(210);

  //Section Titles
  textFont(TitleFont);
  fill(255);
  text("PID Input / Setpoint",(int)ioLeft+10,(int)inputTop-5);
  text("PID Output",(int)ioLeft+10,(int)outputTop-5);


  //GridLines and Titles
  textFont(AxisFont);
  
  //horizontal grid lines
  int interval = (int)inputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,inputTop+i*interval,ioRight-2,inputTop+i*interval);
    text(str((InScaleMax-InScaleMin)/5*(float)(5-i)+InScaleMin),ioRight+5,inputTop+i*interval+4);

  }
  interval = (int)outputHeight/5;
  for(int i=0;i<6;i++)
  {
    if(i>0&&i<5) line(ioLeft+1,outputTop+i*interval,ioRight-2,outputTop+i*interval);
    text(str((OutScaleMax-OutScaleMin)/5*(float)(5-i)+OutScaleMin),ioRight+5,outputTop+i*interval+4);
  }


  //vertical grid lines and TimeStamps
  int elapsedTime = millis();
  interval = (int)ioWidth/vertCount;
  int shift = elapsedTime*(int)ioWidth / windowSpan;
  shift %=interval;

  int iTimeInterval = windowSpan/vertCount;
  float firstDisplay = (float)(iTimeInterval*(elapsedTime/iTimeInterval))/displayFactor;
  float timeInterval = (float)(iTimeInterval)/displayFactor;
  for(int i=0;i<vertCount;i++)
  {
    int x = (int)ioRight-shift-2-i*interval;

    line(x,inputTop+1,x,inputTop+inputHeight-1);
    line(x,outputTop+1,x,outputTop+outputHeight-1);    

    float t = firstDisplay-(float)i*timeInterval;
    if(t>=0)  text(str(t),x,outputTop+outputHeight+10);
  }


  // add the latest data to the data Arrays.  the values need
  // to be massaged to get them to graph correctly.  they 
  // need to be scaled to fit where they're going, and 
  // because 0, 0 is the top left, we need to flip the values.
  // this is easier than having the user stand on their head
  // to read the graph.
  if(millis() > nextRefresh && madeContact)
  {
    nextRefresh += refreshRate;

    for(int i=nPoints-1;i>0;i--)
    {
      InputData[i]=InputData[i-1];
      SetpointData[i]=SetpointData[i-1];
      OutputData[i]=OutputData[i-1];
      CurrentData[i]=CurrentData[i-1];
      WaveformDataE[i] = WaveformDataE[i-1];
    }
    if (nPoints < arrayLength) nPoints++;

    InputData[0] = int(inputHeight)-int(inputHeight*(Input-InScaleMin)/(InScaleMax-InScaleMin));
    SetpointData[0] =int( inputHeight)-int(inputHeight*(Setpoint-InScaleMin)/(InScaleMax-InScaleMin));
    OutputData[0] = int(outputHeight)-int(outputHeight*(Output-OutScaleMin)/(OutScaleMax-OutScaleMin));
    CurrentData[0] = int(outputHeight)-int(outputHeight*(Current-OutScaleMin)/(OutScaleMax-OutScaleMin));
    WaveformDataE[0] = int(outputHeight)-int(outputHeight*(WaveformE-OutScaleMin)/(OutScaleMax-OutScaleMin));
    
  }
  //draw lines for the input, setpoint, and output
  strokeWeight(2);
  for(int i=0; i<nPoints-2; i++)
  {
    int X1 = int(ioRight-2-float(i)*pointWidth);
    int X2 = int(ioRight-2-float(i+1)*pointWidth);
    boolean y1Above, y1Below, y2Above, y2Below;


    //DRAW THE INPUT
    boolean drawLine=true;
    stroke(255,0,0);
    int Y1 = InputData[i];
    int Y2 = InputData[i+1];

    y1Above = (Y1>inputHeight);                     // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>inputHeight);                     // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)inputHeight;                      //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)inputHeight;                   //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)inputHeight;                      //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)inputHeight;       //
    }                                               //

    if(drawLine)
    {
      line(X1,Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE SETPOINT
    drawLine=true;
    stroke(0,255,0);
    Y1 = SetpointData[i];
    Y2 = SetpointData[i+1];

    y1Above = (Y1>(int)inputHeight);                // if both points are outside 
    y1Below = (Y1<0);                               // the min or max, don't draw the 
    y2Above = (Y2>(int)inputHeight);                // line.  if only one point is 
    y2Below = (Y2<0);                               // outside constrain it to the limit, 
    if(y1Above)                                     // and leave the other one untouched.
    {                                               //
      if(y2Above) drawLine=false;                   //
      else if(y2Below) {                            //
        Y1 = (int)(inputHeight);                    //
        Y2 = 0;                                     //
      }                                             //
      else Y1 = (int)(inputHeight);                 //
    }                                               //
    else if(y1Below)                                //
    {                                               //
      if(y2Below) drawLine=false;                   //
      else if(y2Above) {                            //
        Y1 = 0;                                     //
        Y2 = (int)(inputHeight);                    //
      }                                             //
      else Y1 = 0;                                  //
    }                                               //
    else                                            //
    {                                               //
      if(y2Below) Y2 = 0;                           //
      else if(y2Above) Y2 = (int)(inputHeight);     //
    }                                               //

    if(drawLine)
    {
      line(X1, Y1+inputTop, X2, Y2+inputTop);
    }

    //DRAW THE OUTPUT
    drawLine=true;
    stroke(0,0,255);
    Y1 = OutputData[i];
    Y2 = OutputData[i+1];

    y1Above = (Y1>outputHeight);                   // if both points are outside 
    y1Below = (Y1<0);                              // the min or max, don't draw the 
    y2Above = (Y2>outputHeight);                   // line.  if only one point is 
    y2Below = (Y2<0);                              // outside constrain it to the limit, 
    if(y1Above)                                    // and leave the other one untouched.
    {                                              //
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below)                               //
    {                                              //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else                                           //
    {                                              //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine)
    {
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }
    
    //DRAW THE CURRENT
    drawLine=true;
    stroke(0,255,255);
    Y1 = CurrentData[i];
    Y2 = CurrentData[i+1];

    y1Above = (Y1>outputHeight);                   // if both points are outside 
    y1Below = (Y1<0);                              // the min or max, don't draw the 
    y2Above = (Y2>outputHeight);                   // line.  if only one point is 
    y2Below = (Y2<0);                              // outside constrain it to the limit, 
    if(y1Above)                                    // and leave the other one untouched.
    {                                              //
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below)                               //
    {                                              //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else                                           //
    {                                              //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine)
    {
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }    
    //DRAW WaveformDataE
    drawLine=true;
    stroke(125,125,0);
    Y1 = WaveformDataE[i];
    Y2 = WaveformDataE[i+1];

    y1Above = (Y1>outputHeight);                   // if both points are outside 
    y1Below = (Y1<0);                              // the min or max, don't draw the 
    y2Above = (Y2>outputHeight);                   // line.  if only one point is 
    y2Below = (Y2<0);                              // outside constrain it to the limit, 
    if(y1Above)                                    // and leave the other one untouched.
    {                                              //
      if(y2Above) drawLine=false;                  //
      else if(y2Below) {                           //
        Y1 = (int)outputHeight;                    //
        Y2 = 0;                                    //
      }                                            //
      else Y1 = (int)outputHeight;                 //
    }                                              //
    else if(y1Below)                               //
    {                                              //
      if(y2Below) drawLine=false;                  //
      else if(y2Above) {                           //
        Y1 = 0;                                    //
        Y2 = (int)outputHeight;                    //
      }                                            //  
      else Y1 = 0;                                 //
    }                                              //
    else                                           //
    {                                              //
      if(y2Below) Y2 = 0;                          //
      else if(y2Above) Y2 = (int)outputHeight;     //
    }                                              //

    if(drawLine)
    {
      line(X1, outputTop + Y1, X2, outputTop + Y2);
    }   
  }
  strokeWeight(1);
}

void drawButtonArea()
{
  stroke(0);
  fill(100);
  rect(0, 0, ioLeft, windowHeight);
}

void Toggle_AM() {
  if(AMLabel.valueLabel().getText()=="Manual") 
  {
    AMLabel.setValue("Automatic");
  }
  else
  {
    AMLabel.setValue("Manual");   
  }
}


void Toggle_DR() {
  if(DRLabel.valueLabel().getText()=="Direct") 
  {
    DRLabel.setValue("Reverse");
  }
  else
  {
    DRLabel.setValue("Direct");   
  }
}

void Stop()
{
  Byte a = (AMLabel.valueLabel().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.valueLabel().getText()=="Direct")?(byte)0:(byte)1;
  String msg = "gui " + a + " " + d + " " + 
               "0.0" + " " +    //setpoint
               float(InField.getText()) + " " +
               float(OutField.getText()) + " " +
               float(PField.getText()) + " " +
               float(IField.getText()) + " " +
               float(DField.getText()) + 
               "\r\n";
  myPort.write(msg);
  print(msg);
  justSent=true;
  controlP5.getController("SPslider").setValue(0.0);
}

void Send_SP()
{
  Byte a = (AMLabel.valueLabel().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.valueLabel().getText()=="Direct")?(byte)0:(byte)1;
  String msg = "gui " + a + " " + d + " " + 
               float(SPField.getText()) + " " +
               float(InField.getText()) + " " +
               float(OutField.getText()) + " " +
               float(PField.getText()) + " " +
               float(IField.getText()) + " " +
               float(DField.getText()) + 
               "\r\n";
  myPort.write(msg);
  print(msg);
  justSent=true;
}

void Send_PID()
{
  String msg = "pidset " + 
               float(PField.getText()) + " " +
               float(IField.getText()) + " " +
               float(DField.getText()) + 
               "\r\n";
  myPort.write(msg);
  print(msg);
  justSent=true;
}

// Sending Floating point values to the arduino
// is a huge pain.  if anyone knows an easier
// way please let know.  the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Arduino_Old()
{
  float[] toSend = new float[6];

  toSend[0] = float(SPField.getText());
  toSend[1] = float(InField.getText());
  toSend[2] = float(OutField.getText());
  toSend[3] = float(PField.getText());
  toSend[4] = float(IField.getText());
  toSend[5] = float(DField.getText());
  print("tosend "); println(floatArrayToByteArray(toSend));
  Byte a = (AMLabel.valueLabel().getText()=="Manual")?(byte)0:(byte)1;
  Byte d = (DRLabel.valueLabel().getText()=="Direct")?(byte)0:(byte)1;
  myPort.write("gui "); print("gui ");
  myPort.write(a); print(a);
  myPort.write(d); print(d);
  myPort.write(floatArrayToByteArray(toSend)); print(floatArrayToByteArray(toSend));
  myPort.write("\n\r"); print("\n\r");
  justSent=true;
} 


byte[] floatArrayToByteArray(float[] input)
{
  int len = 4*input.length;
  int index=0;
  byte[] b = new byte[4];
  byte[] out = new byte[len];
  ByteBuffer buf = ByteBuffer.wrap(b);
  for(int i=0;i<input.length;i++) 
  {
    buf.position(0);
    buf.putFloat(input[i]);
    for(int j=0;j<4;j++) out[j+i*4]=b[3-j];
  }
  return out;
}


//take the string the arduino sends us and parse it
void serialEvent(Serial myPort)
{
  String read = myPort.readStringUntil(10); //newline
  println(read);

  if(outputFileName!="") output.print(str(millis())+ " "+read);
  String[] s = split(read, " ");

  if (s.length == 11) //10) //9
  {
    //Setpoint = float(s[1]);           // * pull the information
    //Setpoint = float(s[1]);
    //Input = float(s[2]);              //   we need out of the
    //Input = map(Input, -500.0, 500.0, InScaleMin, InScaleMax);
    //Output = float(s[3]); //3 output
    //Output = map(float(s[4]), -500.0, 500.0, InScaleMin, InScaleMax);
    //Current = float(s[9]);//9]);    
    //Current = map(Current, -5000.0, 5000.0, OutScaleMin, OutScaleMax);
    //Current = float(s[9]);//9 current //10 pwm]);
    //Current = map(Current, -3030.0, 3030.0, OutScaleMin, OutScaleMax);
    //Current = map(Current, -5000.0, 5000.0, OutScaleMin, OutScaleMax);
    
    //red
    Setpoint = map(float(s[2]), -1500.0, 1500.0, InScaleMin, InScaleMax);
    //grn
    Input = map(float(s[3]), -1500.0, 1500.0, InScaleMin, InScaleMax);
    //blu
    Output = map(float(s[10]), -500.0, 500.0, OutScaleMin, OutScaleMax);
    //cyan
    Current = map(float(s[1]), -1000.0, 15000.0, OutScaleMin, OutScaleMax);
    
    WaveformE = map(float(s[9]), -1000.0, 15000.0, OutScaleMin, OutScaleMax); 
    
    SPLabel.setValue(s[1]);           //   where it's needed
    InLabel.setValue(s[2]);           //
    OutLabel.setValue(trim(s[3]));    //
    PLabel.setValue(trim(s[4]));      //
    ILabel.setValue(trim(s[5]));      //
    DLabel.setValue(trim(s[6]));      //
    AMCurrent.setValue(trim(s[7]));   //
    DRCurrent.setValue(trim(s[8]));

    CurrentLabel.setValue(trim(s[9]));    //curr pwm out
  //CurrentLabel.setValue(trim(s[9]));    //    
    if(justSent)                      // * if this is the first read
    {                                 //   since we sent values to 
      SPField.setText(trim(s[1]));    //   the arduino,  take the
      InField.setText(trim(s[2]));    //   current values and put
      OutField.setText(trim(s[3]));   //   them into the input fields
      PField.setText(trim(s[4]));     //
      IField.setText(trim(s[5]));     //
      DField.setText(trim(s[6]));     //
     // mode = trim(s[7]);              //
      AMLabel.setValue(trim(s[7]));         //
      //dr = trim(s[8]);                //
      DRCurrent.setValue(trim(s[8]));         //
      justSent=false;                 //
    }                                 //

    if(!madeContact) 
    {
      madeContact=true;
      ConnectLabel.setText("Connected");
    }
  } 
  else 
  {
    println("responce not 9lenght");
    ConnectLabel.setText("DISconnected");
  }
}







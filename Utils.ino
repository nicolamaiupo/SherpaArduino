
float AverageDifferentiator(int cSample, int l1Sample, int l2Sample, int l3Sample, int d1Time, int d2Time, int d3Time){
  if (d3Time != 0){
    float d1 = (cSample-l1Sample)/1.0*d1Time;
    //float d2 = (l1Sample-l2Sample)/1.0*d2Time;
    //float d3 = (l2Sample-l3Sample)/1.0*d3Time;
    //return (d1+d2+d3)/(3*1000); //Tempo in millis
    return d1/1000;
  }else{
    return 0;
  }
}

float LowPassFilter(float value, float last, float cutOff){
  float h = 0.1; //Sample time (approx)
  float Tf= cutOff; //Cutoff frequency
  float a = h/(h+Tf);
  
//  float b = (1-a)*last;
//  float val = value*a+b;
  float val = a*value+(1-a)*last;
  return val;
}




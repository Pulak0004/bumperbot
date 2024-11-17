void leftEncoderCallback()
{
  left_encoder_counter++;
  
  if(digitalRead(left_encoder_b) == HIGH)
  {
    left_encoder_sign = "n";
  }
  else
  {
    left_encoder_sign = "p";
  }
}



void rightEncoderCallback()
{
  right_encoder_counter++;

  if(digitalRead(right_encoder_b) == HIGH)
  {
    right_encoder_sign = "p";
  }
  else
  {
    right_encoder_sign = "n";
  }
}




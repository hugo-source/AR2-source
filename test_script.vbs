Option Explicit

function MainEntry( script_name )
dim x, y, z
dim yaw, pitch, roll
dim delay
dim Sts
dim speed, accel
dim loop_count

  ' set motion parameters
  speed = 1000
  accel = 500
  Sts = AR2Auto.Motion( speed, accel )
  
  ' start position
  delay=1000
  Sts = AR2Auto.Execute( 286.793, 199.847, 138.591, -55.06, 179.97, -124.81, delay )
  loop_count = 1
  Sts = AR2Auto.Notify( "Starting script " & script_name )
  Do
    if ( AR2Auto.Stop_script ) then
      Sts = AR2Auto.Notify( "Stop script requested by operator" )
      Exit do
    end if
    Sts = AR2Auto.Notify( "Pass: " & loop_count )
    ' go to pickup
    Sts = AR2Auto.Execute( -42.055, 398.123, 89.78, 3.538, 179.98, -6.132, delay )
    ' pickup
    Sts = AR2Auto.Execute( -42.038, 397.961, 64.864, 3.621, 179.91, 6.049, delay )
    Sts = AR2Auto.Servo( 45 )
    Sts = AR2Auto.Delay( 1000 )
    Sts = AR2Auto.Servo( 170 )
    ' lift
    Sts = AR2Auto.Execute( -42.044, 398.017, 144.857, 3.605, 179.92, -6.066, delay )
    ' go to set down
    Sts = AR2Auto.Execute( 286.793, 199.847, 138.591, -55.06, 179.97, -124.81, delay )
    ' lower and wait 2 seconds
    Sts = AR2Auto.Execute( 286.793, 199.847, 89.78, -55.06, 179.97, -124.81, 2000 )
    ' drop
    Sts = AR2Auto.Execute( 286.793, 199.847, 64.864, -55.06, 179.97, -124.81, delay )
    Sts = AR2Auto.Servo( 45 )
    Sts = AR2Auto.Delay( 1000 )
    Sts = AR2Auto.Servo( 170 )
    ' after drop, lift arm
    Sts = AR2Auto.Execute( 286.793, 199.847, 138.591, -55.06, 179.97, -124.81, delay )
    loop_count = loop_count + 1
    if ( loop_count > 5 ) then
      Exit Do
    end if
  loop

  x=286.00 
  y=0.00 
  z=440.00 
  yaw=-90.00 
  pitch=180.00 
  roll=-90.00
  delay = 1000
  ' home 
  Sts = AR2Auto.Execute( x, y, z, yaw, pitch, roll, delay )
  Sts = AR2Auto.Notify( "Script " & script_name & " complete." )
  'MsgBox "Script complete..", vbOKOnly, script_name
 
End Function

Option Explicit

function PickUpAndDrop( script_name )
dim x, y, z, x1, y1, z1
dim yaw, pitch, roll
dim delay
dim Sts
dim speed, accel
dim loop_count

  ' pickup point
  delay=1000
  x = 400
  y = -160
  z = 300
  yaw = -90
  pitch = 180
  roll = -90
  
  'drop point
  x1 = 400
  y1 = 160
  z1 = 50
  
  loop_count = 1
  Sts = AR2Auto.Notify( "Starting script " & script_name )
  Sts = AR2Auto.Servo( 170 ) 'close
  Do
    if ( AR2Auto.Stop_script ) then
      Sts = AR2Auto.Notify( "Stop script requested by operator" )
      Exit do
    end if
    Sts = AR2Auto.Notify( "Pass: " & loop_count )
    ' go to pickup point
    Sts = AR2Auto.Execute( x, y, z, yaw, pitch, roll, delay )
    Sts = AR2Auto.Servo( 45 ) 'open
    ' pick up next item
    z = z - 50
    Sts = AR2Auto.Execute( x, y, z, yaw, pitch, roll, delay )
    Sts = AR2Auto.Servo( 170 ) 'close
    Sts = AR2Auto.Delay( 500 )
    ' go to drop point
    z1 = z1 + 50
    Sts = AR2Auto.Execute( x1, y1, z1, yaw, pitch, roll, delay )
    Sts = AR2Auto.Servo( 45 ) 'open
    Sts = AR2Auto.Delay( 500 )

    loop_count = loop_count + 1
    if ( loop_count > 4 ) then
      Exit Do
    end if
  loop

  ' home 
  x=286.00 
  y=0.00 
  z=440.00 
  yaw=-90.00 
  pitch=180.00 
  roll=-90.00
  delay = 0

  Sts = AR2Auto.Execute( x, y, z, yaw, pitch, roll, delay )
  Sts = AR2Auto.Notify( "Script " & script_name & " complete." )
  'MsgBox "Script complete..", vbOKOnly, script_name
 
End Function

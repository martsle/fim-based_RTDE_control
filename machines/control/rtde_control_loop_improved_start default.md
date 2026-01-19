````
#Variables Setup
BeforeStart
  write_output_integer_register(0,1)
  setp:=get_actual_joint_positions() 
  tmp:=[1,1,1,1,1,1]
  command:=1
  Wait: 0.5
  write_output_integer_register(0,0)
  Loop command!= 0
   command = read_input_float_register(6)
  Loop tmp != [0,0,0,0,0,0]
   tmp[0]= read_input_float_register(0)
   tmp[1]= read_input_float_register(1)
   tmp[2]= read_input_float_register(2)
   tmp[3]= read_input_float_register(3)
   tmp[4]= read_input_float_register(4)
   tmp[5]= read_input_float_register(5)
  rtde_set_watchdog("input_int_register_0", 1, "STOP")
#Robot Program 
 setp:=get_actual_joint_positions()
 tmp:=[0,0,0,0,0,0]
 tmp[0]= read_input_float_register(0)
 tmp[1]= read_input_float_register(1)
 tmp[2]= read_input_float_register(2)
 tmp[3]= read_input float_register(3)
 tmp[4]= read_input_float_register(4)
 tmp[5]= read_input_float register(5)
 x=norm(tmp)
 If x
  setp:=tmp
  write_output_integer_register(0, 0)
 servoj(setp, 0, 0, 0,02, 0.1, 300)

````

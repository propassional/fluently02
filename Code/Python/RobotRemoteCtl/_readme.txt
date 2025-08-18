Apply 3D algorithms, evaluate results

x https://video-converter.com from Mattia, for changing video formats
x Remember to switch off scanner at night

x In order to have a repeatable setup it is necessary to send the robot to the start point with the correct joints?
Se ha già fatto la pirouette, la fa ancora?

x Protocol
    x Save scan parameters: time, manual control, file name of the positions
    x Save cartesian points xyz alfa beta jamma, j1-j6

x Scan quality
x Check distance
x Time how much?

x Pirouette problem
    v Pirouette è positiva? si
    v Inizio file originale: J6: -143 ho aggiunto +360 facendolo ruotare a mano, J6 = 216
     Non appena ha caricato il primo punto, ha fatto una -pirouette per tornare come era prima
    Pirouette moment: -11.883712768554688,152.7984619140625,114.64078521728516,143.7983856201172,-86.51761627197266,-46.71105194091797

Startup joint positions:
J1 120
J2 46
J3 -18
J4 -3
J5 -70
J6 -142

Cartesian coordinates
x, y, z, roll, pitch, yaw (gradi)

Cartesian control
the robot starts from the current position in joints found at the beginning of operation,
and by itself calculates the joints to arrive at the required cartesian position

Pirouette problem
Find pirouette points: from index 56 to 57: z revolves from -178.76 to 178.89

send_command

self.control_time = 0.2 sec ogni quanto scrivi il punto

Activate robot, launch fanuc remote_ctrl program, launch move_robot
How to start ROS2: D:\Banfi\Github\Fluently\Code\Python\ROS2\_readme.txt
How to start ROS2: D:\Banfi\Github\Fluently\Code\Python\ROS2\_readme.txt

Intents used: D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Output\Intents\_Released

Source ROS2 with cmd: C:\Users\operator>call C:\dev\ros2_humble\local_setup.bat

Login bestione: rfluently1 / 111

Verifica che sei su ROS2 di Fluently
Apri cmd
Source ROS2: >call C:\dev\ros2_humble\local_setup.bat 
>ros2 topic list
Qui stava girando un tot di roba di Quattrini
/camera_232700107/color/camera_info
/camera_232700107/color/image_raw
/camera_232800003/color/camera_info
/camera_232800003/color/image_raw
/camera_232902014/aligned_depth_to_color/camera_info
/camera_232902014/aligned_depth_to_color/image_raw
/camera_233000203/aligned_depth_to_color/camera_info
/camera_233000203/aligned_depth_to_color/image_raw
/clicked_point
/convexhull/mesh_marker
/convexhull/mesh_marker_array
/fluently/tts
/goal_pose
/gui_output
/hfluently/PA_transcription
/initialpose
/nlu_output
/parameter_events
/rfluently/simple_intent
/rosout
/sm_state
/tf
/tf_static

>ros2 node list 
/mqtt_client
/nlu_node
/node1
/node2
/node3
/node4
/rviz
/static_transform_publisher_CKSVwE7O9oeYwwii
/static_transform_publisher_jdhivaPgNF0dqLNb
/talker
/transform_listener_impl_60a78166a8d0


Step0: far girare il sw di Rocco
x Rocco doc on NLU module:
	- Per lanciarlo fai right mouse click poi run as a program sul file ./build_docker.sh all'interno della directory /Desktop/natural-language-understanding
	=> il nodo parte in automatico, lo vedi come nlu_node con >ros2 node list
		- Con Andrea Bussolan avevamo cambiato un parametro per permettergli di usare la lan vera, altrimenti al di fuori non vedi nulla > Ora è fissato, tutto ok
	- Per pubblicare lo stato corrente della state_machine pf utilizza un topic con il seguente nome: sm_state
	  Per leggere: ros2 topic echo /sm_state, resta bloccato se nessuno lo aggiorna (non mostra il vecchio valore!)
	  Per scrivere: 
	  
		Utilizza una semplice stringa come tipo di messaggio con il seguente formato 'TabNode' e.g. InstructionsStart, InstructionsReady or PreviewBasicArtecStudioIsOff
		Come output sul topic gui_output troverai in formato stringa il nome della transizione che l'operatore vorrebbe eseguire
	- Tutte queste informazioni le trovi nel branch 'pa' su gitlab (sono tutte documentate):
		https://gitlab-core.supsi.ch/dti-isteps/armlab/fluently/natural-language-understanding/-/tree/pa?ref_type=heads
		

Step1: collegare la voce
Alternativa A]
1) Chiedi hfluently a Corrado, che lo collega via lan a fluently
2) Chiedi a Corrado di tirar su il DB di fluently per login ecc
Le cose di Corrado sul bestione dovrebbero già girare, di default
Alternativa B]
Collega le cuffie al PC, fai girare whisper, 
poi dopo pubblica la trascrizione whisper su ROS2 con cmd sourcato, cosi Rocco puo' leggerla:
ros2 topic pub hfluently/PA_transcription std_msgs/msg/String "{data: '{\"transcription\": \"MyEBuddyMessage\", \"timestamp\": 1}'}" --once

Step2: Pubblicare a Rocco in quale stato si trova EBuddy: sm_state è TabName cat State, ad es. InstructionsReady
ros2 topic pub /sm_state std_msgs/msg/String "data: 'InstructionsReady'" -r 2

Step3: Rocco pubblica il suo intent sotto gui_output, leggilo:
ros2 topic echo /gui_output



ros2 run demo_nodes_cpp talker
For reading: ros2 topic echo /chatter

ros2 interface list
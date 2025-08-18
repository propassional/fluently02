Ciao Michele, 

ti riassumo le indicazioni per l'utilizzo del modulo nlu:
pwd fluently: 111
Apri terminal con ctrl+alt+t
$cd /home/fluently1/Desktop

per lanciarlo usi ./build_docker.sh all'interno della directory /Desktop/natural-language-understanding e il nodo parte in automatico.

Per pubblicare lo stato corrent della state_machine puoi utilizzare un topic con il seguente nome: sm_state 

Utilizza una semplice stringa come tipo di messaggio con il seguente formato 'TabNode' e.g. InstructionsStart, InstructionsReady or PreviewBasicArtecStudioIsOff.

Come output sul topic gui_output troverai in formato stringa il nome della transizione che l'operatore vorrebbe eseguire.

Tutte queste informazioni le trovi nel branch 'pa' su gitlab (sono tutte documentate):

https://gitlab-core.supsi.ch/dti-isteps/armlab/fluently/natural-language-understanding/-/tree/pa?ref_type=heads

Se avessi domande o dubbi sentiamoci pure. 

Rocco
Concorrenti: https://www.univetar.com/it/l/supporto-remoto/?utm_term=&utm_campaign=EndUser_Remoto_ITA&utm_source=google&utm_medium=display&utm_content=&hsa_acc=6602746198&hsa_cam=12508845550&hsa_grp=118580267186&hsa_ad=504820510025&hsa_src=d&hsa_tgt=&hsa_kw=&hsa_mt=&hsa_net=adwords&hsa_ver=3&gclid=EAIaIQobChMIma7suLyMhAMV85D9Bx2YugCFEAEYASAAEgItnfD_BwE

******************************************************************************************************************************
************************ Setup Realwear **************************************************************************************
******************************************************************************************************************************
https://shop.realwear.com/collections/hmt/products/realwear-navigator-520 3'300 $

Le cuffie assegnate alla use case PA 
Dal 23.04.24 Corrado mi ha assegnato le Fluently2
Originariamente erano le cuffie Fluently1, 
ma se puoi usa le Fluently0 che hanno uno schermo piu' leggibile,
ma ricorda che hanno il plug USB vecchio, e non USB-C

Switch on Realwear:
Se il video è spento, e Realwear è in standby, basta premere una volta il tasto quadrato nero, si riattiva subito
Se il video è spento, e Realwear è spento, premere a lungo il tasto quadrato nero, e tenerlo premuto 4 secondi circa

Charge Realwear:
Quando metti il  Realwear in carica devi vedere il led rosso che si accende, altrimenti non si sta caricando

Pair Realwear with rfluently1
Check se Realwear è connesso: 
Sul Realwear normale: da qualsiasi parte tu sia pronuncia "navigate back" > poi 
> my programs > settings > connected devices: 
Sotto il menu "currenctly connected" si deve leggere rfluently1 e il numero associato e dire "select item nr"
	altrimenti dire ora "Pair new device", poi select item "x"
(Sul Realwear nostro (tarocco): my programs > my controls > bluetooth > bluetooth settings: you see connected devices)
su rfluently: click on main ubuntu button poi settings poi sound e accept pairing
After pairing, on rfluently remember to set the device "Navigator-5XX" as audio device, since Ubuntu22 does it not automatically as windows does

Quando nel display vedi un comando con un numero cerchiato:
Say "navigate three" or "select item three" > va al nr 3

Other commands: show help, navigate back, 
my programs > settings > display > screen timeout

Settings > display

Come riattivare Realwear: 
1) Wear them, accenderle, senti i suoni di avvio (con fluently1, ma con fluently0 non mi sembra si sentano suoni), 
entrarci e connettersi al BT della beast (cfr doc Realwear su ArtecPC.D:\)
Le cuffie si chiamano tutte "Navigator-5XX", quindi se passi da una all'altra il nome resta lo stesso ma OS le discerne, quindi cancella quella vecchia!
##########################################################
my programs > my controls > bluetooth > bluetooth settings> pair new device > select item three (for example)
##########################################################
   => Verifica che in "connected devices" ci sia rfluently
   => Verifica che l'audio sia settato sulle Realwear, cioè Handsfree-Navigator SXX
2) Thinkpad>roscore non serve rilanciarlo, e non va mai fermato altrimenti cade il bridge
3) Se non va forse devi rilanciare il nodo di whisper che si è staccato da BT
4) Launch ArtecPC>server, poi Thinkpad>Client relaunch

Ripetere questo punto ad ogni restart della demo
5) ThinkCentre launch client: /home/fluently1/supsi_ws>roslaunch artec normal.launch 
since client is in blocking mode, ctrl-c works only if server is down
Use "terminator" terminal

Shutdown Realwear: 
say "navigate home" (se non sei gia' nella schermata iniziale), my controls > power options > power down
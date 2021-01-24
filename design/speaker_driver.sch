<Qucs Schematic 0.0.22>
<Properties>
  <View=-89,-550,2459,1578,1.4641,178,677>
  <Grid=10,10,1>
  <DataSet=speaker_driver.dat>
  <DataDisplay=speaker_driver.dpl>
  <OpenDisplay=1>
  <Script=speaker_driver.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Titolo>
  <FrameText1=Disegnato da:>
  <FrameText2=Data:>
  <FrameText3=Versione:>
</Properties>
<Symbol>
</Symbol>
<Components>
  <IProbe I1 1 200 130 -5 16 0 0>
  <C_SPICE C2 1 380 160 17 -26 0 1 "0.68u" 1 "" 0 "" 0 "" 0 "" 0>
  <GND * 1 480 190 0 0 0 0>
  <GND * 1 380 190 0 0 0 0>
  <C_SPICE C1 1 340 130 -24 -65 0 2 "100u" 1 "" 0 "" 0 "" 0 "" 0>
  <GND * 1 140 460 0 0 0 0>
  <Vac V3 1 140 210 18 -26 0 1 "1.65 V" 1 "1 KHz" 0 "0" 0 "0" 0>
  <Eqn Eqn1 1 390 380 -31 19 0 0 "K=dB(Vsp.v/Vi.v)" 1 "yes" 0>
  <Vpulse V2 1 140 290 18 -26 0 1 "-1.65 V" 1 "1.65 V" 1 "0.25us" 1 "0.75 us" 1 "1 ns" 0 "1 ns" 0>
  <Vdc V1 1 140 420 18 -26 0 1 "1.65 V" 1>
  <L_SPICE L1 1 560 170 -26 10 0 0 "330n" 1 "" 0 "" 0 "" 0 "" 0>
  <R_SPICE R2 1 560 130 -26 -61 0 2 "16" 1 "" 0 "" 0 "" 0 "" 0>
  <.TR TR1 1 1940 1030 0 83 0 0 "lin" 1 "0" 1 "2 us" 1 "1024" 1 "Trapezoidal" 0 "2" 0 "1 ns" 0 "1e-16" 0 "150" 0 "0.001" 0 "1 pA" 0 "1 uV" 0 "26.85" 0 "1e-3" 0 "1e-6" 0 "1" 0 "CroutLU" 0 "no" 0 "yes" 0 "0" 0>
  <.AC AC1 1 1080 1030 0 49 0 0 "log" 1 "1 Hz" 1 "100 KHz" 1 "1024" 1 "no" 0>
  <R R1 1 280 130 -18 12 0 0 "100" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
</Components>
<Wires>
  <370 130 380 130 "" 0 0 0 "">
  <230 130 250 130 "" 0 0 0 "">
  <140 130 170 130 "" 0 0 0 "">
  <140 130 140 180 "" 0 0 0 "">
  <140 240 140 260 "" 0 0 0 "">
  <140 450 140 460 "" 0 0 0 "">
  <140 320 140 390 "" 0 0 0 "">
  <380 130 530 130 "Vsp" 440 70 107 "">
  <480 170 480 190 "" 0 0 0 "">
  <480 170 530 170 "" 0 0 0 "">
  <590 130 590 170 "" 0 0 0 "">
  <140 130 140 130 "Vi" 107 70 0 "">
</Wires>
<Diagrams>
  <Rect 810 478 666 388 3 #c0c0c0 1 10 1 1 1 100000 1 -0.000506651 0.002 0.018 1 -0.0195392 0.05 0.25 315 0 225 "f [Hz]" "I1 [A]" "Vsp [V]">
	<"ngspice/ac.i(vi1)" #ff0000 0 3 0 0 0>
	<"ngspice/ac.v(vsp)" #0000ff 0 3 0 0 1>
  </Rect>
  <Rect 800 937 688 367 3 #c0c0c0 1 10 1 1 1 100000 1 -42.2532 5 -14.9387 1 -1 0.2 1 315 0 225 "f [Hz]" "Vsp/Vi [dB]" "">
	<"ngspice/ac.k" #0000ff 0 3 0 0 0>
  </Rect>
  <Rect 1637 469 775 386 3 #c0c0c0 1 00 1 0 2e-07 2e-06 1 -0.00347917 0.005 0.0363444 1 -1 0.2 1 315 0 225 "T [s]" "I1 [A]" "">
	<"ngspice/tran.i(vi1)" #ff0000 0 3 0 0 0>
  </Rect>
  <Rect 1640 939 775 369 3 #c0c0c0 1 00 1 0 2e-07 2e-06 1 -0.0139834 0.02 0.153818 1 -1 0.2 1 315 0 225 "T [s]" "Vsp [V]" "">
	<"ngspice/tran.v(vsp)" #0000ff 0 3 0 0 0>
  </Rect>
</Diagrams>
<Paintings>
  <Rectangle 510 60 100 180 #000000 0 1 #c0c0c0 1 0>
  <Text 530 240 12 #000000 0 "Speaker">
</Paintings>

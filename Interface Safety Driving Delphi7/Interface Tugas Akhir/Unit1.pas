unit Unit1;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls, TeEngine, Series, ExtCtrls, TeeProcs, Chart, ComCtrls,
  CPort, Grids;

type
  TForm1 = class(TForm)
    cmprt1: TComPort;
    cmdtpckt1: TComDataPacket;
    pgc1: TPageControl;
    ts1: TTabSheet;
    cht1: TChart;
    lnsrsSeries1: TLineSeries;
    lnsrsSeries2: TLineSeries;
    lnsrsSeries3: TLineSeries;
    cht2: TChart;
    lnsrsSeries5: TLineSeries;
    lnsrsSeries6: TLineSeries;
    lnsrsSeries7: TLineSeries;
    btn1: TButton;
    btn2: TButton;
    btn3: TButton;
    edt1: TEdit;
    ts2: TTabSheet;
    cht3: TChart;
    lnsrsSeries9: TLineSeries;
    cht5: TChart;
    lnsrsSeries4: TLineSeries;
    lnsrsSeries8: TLineSeries;
    lnsrsSeries11: TLineSeries;
    btn6: TButton;
    ts4: TTabSheet;
    cht4: TChart;
    lnsrsSeries10: TLineSeries;
    lnsrsSeries20: TLineSeries;
    lnsrsSeries21: TLineSeries;
    lnsrsSeries22: TLineSeries;
    lnsrsSeries23: TLineSeries;
    lnsrsSeries24: TLineSeries;
    lnsrsSeries25: TLineSeries;
    lnsrsSeries26: TLineSeries;
    lnsrsSeries27: TLineSeries;
    lnsrsSeries28: TLineSeries;
    lnsrsSeries29: TLineSeries;
    ts5: TTabSheet;
    cht7: TChart;
    cht8: TChart;
    cht9: TChart;
    lnsrsSeries30: TLineSeries;
    lnsrsSeries31: TLineSeries;
    lnsrsSeries32: TLineSeries;
    lnsrsSeries33: TLineSeries;
    lnsrsSeries34: TLineSeries;
    ts6: TTabSheet;
    mmo1: TMemo;
    lnsrsSeries12: TLineSeries;
    dlgSave1: TSaveDialog;
    btn7: TButton;
    cht6: TChart;
    lnsrsSeries13: TLineSeries;
    lbl1: TLabel;
    edt2: TEdit;
    edt3: TEdit;
    lnsrsSeries14: TLineSeries;
    lnsrsSeries15: TLineSeries;
    edt4: TEdit;
    lnsrsSeries16: TLineSeries;
    ts3: TTabSheet;
    cht10: TChart;
    lnsrsSeries17: TLineSeries;
    lnsrsSeries18: TLineSeries;
    lnsrsSeries19: TLineSeries;
    lnsrsSeries35: TLineSeries;
    ts7: TTabSheet;
    cht11: TChart;
    lnsrsSeries36: TLineSeries;
    lnsrsSeries37: TLineSeries;
    lnsrsSeries38: TLineSeries;
    lnsrsSeries39: TLineSeries;
    lnsrsSeries40: TLineSeries;
    lbl3: TLabel;
    lbl4: TLabel;
    edt5: TEdit;
    lnsrsSeries41: TLineSeries;
    edt6: TEdit;
  //  procedure FormCreate(Sender: TObject; var Action: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormClose(Sender: TObject; var Action: TCloseAction);
    procedure atur_chart;
    procedure resettampilan;
    procedure btn1Click(Sender: TObject);
    procedure btn2Click(Sender: TObject);
    procedure btn3Click(Sender: TObject);
    procedure DP1(Sender: TObject; const Str: string);
    procedure btn4Click(Sender: TObject);
    procedure btn5Click(Sender: TObject);
    procedure btn6Click(Sender: TObject);
    procedure btn7Click(Sender: TObject);
  private
    { Private declarations }
  public
    { Public declarations }
  end;

var
  Form1: TForm1;
 // Form2: TForm2;
  ndata,i:integer;
  cek,dat:string;
  sta,sto:cardinal;
  flag:boolean;
  fs,tmax,t,tt,t1,tb,dt:Extended;
  //tarr,real,tarr1:real;
  ix,input1,input2,barpos1,barpos2,u1,temp,low1,low2,low3,low4,low5,loww,medd,highh:Extended;
//  procedure FormClose(Sender: TObject; var Action: TCloseAction);

  //data

  ts  : array [-10..3000000] of Extended;
//
  dtH,dtL,cy:integer;
  ecg,acc1x,acc1z,acc2x,acc2z,acc3x,acc3z,acc4x,acc4z,gy1y,gy2y,gy3y,gy4y:array [-10..3000000] of Extended;
  RUac,DPac:array [-10..3000000] of extended;
  RUkal,DPkal:array [-10..3000000] of extended;
  vel:array [-10..3000000] of extended;
  RPac,RLac,DPAac,DLac:array [-10..3000000] of extended;
  RPkal, miud_s, miud_m, miud_l, miud_vl,RLkal,DPAkal,DLkal,Elpf,Ehpf,deriv,squar,mav, mav1,thr1,rr,hr,qrs,durasi,outf:array [-10..3000000] of extended;
  gyroRP,gyroRL,gyroDPA,gyroDL,Elpf1,max1,max2,max3,max4,durasi1:array [-10..3000000] of extended;
  Vecg, Vecg1,VRPx,VRPz,VRLx,VRLz,VDPAx,VDPAz,VDLx,VDLz,VgyroRP,VgyroRL,VgyroDPA,VgyroDL:array [-10..3000000] of Real;
  dwt1,dwt2,dwt3,dwt4,dwt5,dwt6,dwt7,velocity,miuk_low,miuk_med,miuk_high,miuh_low,miuh_med,miuh_high,miup_low,miup_med,miup_high:array [-10..3000000] of extended;
  small_bt,med_bt,large_bt,neglarge_bt,negmed_bt,negmed_bt1,negsmall_bt,zero_bt,possmall_bt,posmed_bt,poslarge_bt,outnvl_bt,outnl_bt,outnm_bt,outns_bt,outzero_bt,outps_bt,outpm_bt,outpl_bt,outpvl_bt,
//  rule,akhir:array[-1000..3000000]of real;
  kondisi : array [-10..3000000] of string;
//


implementation

{$R *.dfm}


procedure TForm1.FormClose(Sender: TObject; var Action: TCloseAction);
begin
 cmprt1.close;
 //cmprt1.Open;
end;

function Triangle(a:real;b:real;c:real;x:real):extended;
begin
  if x<=a then Triangle:=0
  else if (x>=a) and (x<b) then Triangle:=(x-a)/(b-a)
  else if (x>=b) and (x<c) then Triangle:=(c-x)/(c-b)
  else if x>=c then Triangle:=0;
end;

function Trapezoid(a:real;b:real;c:real;d:real;x:real):extended;
begin
  if x<=a then Trapezoid:=0
  else if (x>=a) and (x<=b) then Trapezoid:=(x-a)/(b-a)
  else if (x>=b) and (x<=c) then Trapezoid:=1
  else if (x>=c) and (x<=d) then Trapezoid:=(d-x)/(d-c)
  else if x>=d then Trapezoid:=0;
end;

procedure TForm1.FormCreate(Sender: TObject);
begin
//  comport1.Open;
// for i := 0 to 1000 do
//  begin
//    ix:=i/10;
//    neglarge_bt[i]:=trapezoid(-360,-360,30,40,ix);
//    negmed_bt[i]:=triangle(30,45,60,ix);
//    negmed_bt1[i]:=triangle(50,65,80,ix);
//    negsmall_bt[i]:=trapezoid(70,80,140,140,ix);
//    lnsrsSeries36.AddXY(ix,neglarge_bt[i]);
//    lnsrsSeries37.AddXY(ix,negmed_bt[i]);
//    lnsrsSeries38.AddXY(ix,negmed_bt1[i]);
//    lnsrsSeries39.AddXY(ix,negsmall_bt[i]);
//  end;
// for i := -460 to 460 do
// begin
//    ix:=i/10;
//    small_bt[i]:=trapezoid(-46,-46,-20,-10.5,ix);
//    med_bt[i]:=triangle(-15,0,15,ix);
//    large_bt[i]:=trapezoid(10.5,20,46,46,ix);
//    Series31.AddXY(ix,small_bt[i]);
//    Series32.AddXY(ix,med_bt[i]);
//    Series33.AddXY(ix,large_bt[i]);
//  end;
//  for i := -482 to 482 do
//  begin
//    ix:=i/10;
//    outnl_bt[i]:=triangle(-48.2,-48.2,-31.8,ix);
//    outnm_bt[i]:=triangle(-41.5,-31.8,-18.84,ix);
//    outns_bt[i]:=triangle(-27.38,-14.7,-4.7,ix);
//    outzero_bt[i]:=triangle(-11.36,0,11.36,ix);
//    outps_bt[i]:=triangle(4.7,14.7,27.38,ix);
//    outpm_bt[i]:=triangle(18.84,31.8,41.5,ix);
//    outpl_bt[i]:=triangle(31.8,48.2,48.2,ix);
//    Series45.AddXY(ix,outnl_bt[i]);
//    Series46.AddXY(ix,outnm_bt[i]);
//    Series47.AddXY(ix,outns_bt[i]);
//    Series44.AddXY(ix,outzero_bt[i]);;
//    Series49.AddXY(ix,outps_bt[i]);
//    Series50.AddXY(ix,outpm_bt[i]);
//    Series51.AddXY(ix,outpl_bt[i]);
//  end;
end;


procedure TForm1.btn1Click(Sender: TObject);
begin
  if btn1.Caption = 'Open Port' then
  begin
    btn1.Caption := 'Close Port';
    cmprt1.Open;
    if (cmprt1.Connected = True) then
    begin
    //Flag:=True;
    resettampilan;
    atur_chart;
    edt1.Text := cmprt1.Port;
    i := 0;
    t := 0;
    fs := 250;
    dt := 1/fs;
    end;
  end
  else if btn1.Caption = 'Close Port' then
  begin
    btn1.Caption := 'Open Port';
    cmprt1.Close;
    if (cmprt1.Connected = False) then
    begin
    //Flag:=False;
      edt1.Clear;
    end;
  end;
end;

procedure TForm1.btn2Click(Sender: TObject);
begin
 if btn2.Caption = 'Start' then
  begin
    btn2.Caption := 'Stop';
    resettampilan;
    atur_chart;
    i := 0;
    t := 0;
    fs := 250;
    dt := 1/fs;
    tmax := 6;
    Flag := True;
    btn1.Enabled := False;
    cmprt1.WriteStr('a');
  end
  else if btn2.Caption = 'Stop' then
  begin
    if (cmprt1.Connected = True) then
    begin
      cmprt1.WriteStr('b');
    end;
    btn2.Caption := 'Start';
    btn1.Enabled := True;
    Flag := False;
    i := 0;
    t := 0;
  end;
end;

procedure TForm1.btn3Click(Sender: TObject);
begin
Application.Terminate;
end;

procedure TForm1.atur_chart;
begin
    tmax:=6;
    cht1.BottomAxis.Minimum:=round(t);cht1.BottomAxis.Maximum:=round(t)+tmax;
    cht2.BottomAxis.Minimum:=round(t);cht2.BottomAxis.Maximum:=round(t)+tmax;
    cht3.BottomAxis.Minimum:=round(t);cht3.BottomAxis.Maximum:=round(t)+tmax;
    cht4.BottomAxis.Minimum:=round(t);cht4.BottomAxis.Maximum:=round(t)+tmax;
    cht5.BottomAxis.Minimum:=round(t);cht5.BottomAxis.Maximum:=round(t)+tmax;
    cht6.BottomAxis.Minimum:=round(t);cht6.BottomAxis.Maximum:=round(t)+tmax;
    cht7.BottomAxis.Minimum:=round(t);cht7.BottomAxis.Maximum:=round(t)+tmax;
    cht8.BottomAxis.Minimum:=round(t);cht8.BottomAxis.Maximum:=round(t)+tmax;
    cht9.BottomAxis.Minimum:=round(t);cht9.BottomAxis.Maximum:=round(t)+tmax;
    cht10.BottomAxis.Minimum:=round(t);cht10.BottomAxis.Maximum:=round(t)+tmax;

end;

procedure TForm1.resettampilan;
begin
    lnsrsSeries1.clear;
    lnsrsSeries2.clear;
    lnsrsSeries3.clear;
    lnsrsSeries5.clear;
    lnsrsSeries6.clear;
    lnsrsSeries7.clear;
    lnsrsSeries9.clear;
    lnsrsSeries10.clear;
    atur_chart;
end;


procedure TForm1.DP1(Sender: TObject; const Str: string);
var
  m : integer;
begin
  if Flag = True then
  begin
    ts[i] := t;

    //==============Sensor System================//

    if Str[1] = 'e' then
    begin
      //---------------Raw Sensor Sheet 1 dan 2---------------//
      acc1x[i]    := ord(Str[2])*100+ord(Str[3]);
      acc1z[i]    := ord(Str[4])*100+ord(Str[5]);
      gy1y[i]     := ord(Str[6])*100+ord(Str[7]);
      ecg[i]      := ord(Str[8])*100+ord(Str[9]);

      RPac[i]     := (((ord(Str[10])*100+ord(Str[11]))/10)-360);
      RPkal[i]    := (((ord(Str[12])*100+ord(Str[13]))/10)-360);
      velocity[i] := (((ord(Str[14])*100+ord(Str[15])))-1000);
//      velocity[i] := (((ord(Str[14])*100+ord(Str[15])))-1000);
//      gyroRP[i]   := (ord(str[16])*100+ord(str[17])/10+ord(str[18])-360);

      qrs[i]     := ord(Str[16]);


      Vecg1[i]    := (((ord(Str[17])*100+ord(Str[18]))/100)-50);
      hr[i]       := (((ord(Str[19])*100+ord(Str[20]))/100));
      durasi[i]   := (((ord(Str[21])*100+ord(Str[22]))/100)-20);
      outf[i]     := (((ord(Str[23])*100+ord(Str[24]))/10)-200);
//      durasi1[i]  := (((ord(Str[25])*100+ord(Str[26]))/100)-20);
//
//      miup_low[i] := (((ord(Str[23])*100+ord(Str[24]))/100)-50);
//      miup_high[i]:= (((ord(Str[25])*100+ord(Str[26]))/100)-50);
//
//      miud_s[i] := (((ord(Str[27])*100+ord(Str[28]))/100)-50);
//      miud_m[i] := (((ord(Str[29])*100+ord(Str[30]))/100)-50);
//      miud_l[i] := (((ord(Str[31])*100+ord(Str[32]))/100)-50);
//      miud_vl[i]:= (((ord(Str[33])*100+ord(Str[34]))/100)-50);
//
//      miuk_low[i] := (((ord(Str[35])*100+ord(Str[36]))/100)-50);
//      miuk_high[i]:= (((ord(Str[37])*100+ord(Str[38]))/100)-50);
//
//      max1[i] := (((ord(Str[39])*100+ord(Str[40]))/100)-50);
//      max2[i]:= (((ord(Str[41])*100+ord(Str[42]))/100)-50);
//
//      max3[i] := (((ord(Str[43])*100+ord(Str[44]))/100)-50);
//      max4[i]:= (((ord(Str[45])*100+ord(Str[46]))/100)-50);
//
//      Elpf[i]      := (((ord(Str[21])*100+ord(Str[22]))/10)-100);
//      Ehpf[i]      := (((ord(Str[23])*100+ord(Str[24]))/100)-50);
//      deriv[i]     := (((ord(Str[25])*100+ord(Str[26]))/10)-200);
//      squar[i]     := ((ord(Str[27])*100+ord(Str[28]))-1000);
//      mav[i]       := (((ord(Str[29])*100+ord(Str[30]))/10)-100);
//      mav1[i]      := (((ord(Str[31])*100+ord(Str[32]))/10)-100);
//      thr1[i]      := (((ord(Str[33])*100+ord(Str[34]))/10)-200);
//     // Str[38] := 'f' ;
//
//      miuh_low[i] := (((ord(Str[38])*100+ord(Str[39]))/100)-50);
//      miuh_med[i] := (((ord(Str[40])*100+ord(Str[41]))/100)-50);
//      miuh_high[i]:= (((ord(Str[42])*100+ord(Str[43]))/100)-50);
//
//      miup_low[i] := (((ord(Str[44])*100+ord(Str[45]))/100)-50);
//      miup_med[i] := (((ord(Str[46])*100+ord(Str[47]))/100)-50);
//      miup_high[i]:= (((ord(Str[48])*100+ord(Str[49]))/100)-50);
//
//      miuk_low[i] := (((ord(Str[50])*100+ord(Str[51]))/100)-50);
//      miuk_med[i] := (((ord(Str[52])*100+ord(Str[53]))/100)-50);
//      miuk_high[i]:= (((ord(Str[54])*100+ord(Str[55]))/100)-50);
//        durasi[i]    := (((ord(Str[35])*100+ord(Str[36]))/100)-20);
//
//        outf[i]:= (((ord(Str[37])*100+ord(Str[38]))/10)-200);

      if outf[i]<=30 then kondisi[i]:= 'Awake'
      else if (outf[i]>30) and (outf[i]<=40) then kondisi[i]:='Awake & Drowsy1'
      else if (outf[i]>40) and (outf[i]<=50) then kondisi[i]:='Drowsy1'
      else if (outf[i]>50) and (outf[i]<=60) then kondisi[i]:='Drowsy1 & Drowsy2'
      else if (outf[i]>60) and (outf[i]<=70) then kondisi[i]:='Drowsy2'
      else if (outf[i]>70) and (outf[i]<=80) then kondisi[i]:='Drowsy2 & Sleepness'
      else if (outf[i]>80) and (outf[i]<=100) then kondisi[i]:='Sleepness' ;

      edt5.Text:=kondisi[i];
      edt6.Text:=kondisi[i];

     // edit2.Text:=floatToStr(ecg[i]);
      //---------------Data Voltage---------------//
      VRPx[i]       := (acc1x[i]*3.3)/4095;
      VRPz[i]       := (acc1z[i]*3.3)/4095;
      VgyroRP[i]    := (gy1y[i]*3.3)/4095;
      Vecg[i]       := (ecg[i]*3.3)/4095;

//      if i>1 then
//      Vel[i] := (Rpac[i]-RPac[i-1])*250.0;



     //  edt2.Text:=floatToStr(Abs(Rpac[i]));
     //  edt3.Text:=floatToStr(velocity[i]);
       edt4.Text:=floatToStr(hr[i]);
      //Kalau mau kirim stim jadi harus ada rumus
      //dapetin nilai duty (di pwm boost nanti dapat tegangannya

//      mmo1.lines.add(floattostr(i)+#9+floattostr(acc1x[i])+#9+floattostr(acc1z[i])+#9+floattostr(gy1y[i])+#9+
//      floattostr(ecg[i])+#9+floattostr(Abs(RPac[i]))+#9+floattostr(abs(RPkal[i]))+#9+floattostr(velocity[i])+#9+floattostr(Abs(vel[i]))+#9+floattostr(Vecg1[i])+#9+floattostr(qrs[i])+#9+
//      floattostr(hr[i])+#9+floattostr(elpf[i])+#9+floattostr(ehpf[i])+#9+floattostr(deriv[i])+#9+floattostr(squar[i])+#9+floattostr(mav[i])+#9+floattostr(mav1[i])+#9+floattostr(thr1[i]));
//
 mmo1.lines.add(floattostr(i)+#9+floattostr(acc1x[i])+#9+floattostr(acc1z[i])+#9+floattostr(gy1y[i])+#9+
      floattostr(ecg[i])+#9+floattostr(Vecg1[i])+#9+floattostr(abs(RPac[i]))+#9+floattostr(abs(RPkal[i]))+#9+floattostr(velocity[i])+#9+floattostr(durasi[i])+#9+floattostr(hr[i])+#9+floattostr(outf[i])+#9+kondisi[i]);

       lnsrsSeries20.addxy(ts[i],Vecg1[i]);
       lnsrsSeries21.addxy(ts[i],qrs[i]);
       lnsrsSeries22.addxy(ts[i],hr[i]);
//       lnsrsSeries23.addxy(ts[i],elpf[i]);
//       lnsrsSeries24.addxy(ts[i],ehpf[i]);
//       lnsrsSeries25.addxy(ts[i],deriv[i]);
//       lnsrsSeries26.addxy(ts[i],squar[i]);
//       lnsrsSeries27.addxy(ts[i],mav[i]);
//       lnsrsSeries28.addxy(ts[i],mav1[i]);
//       lnsrsSeries29.addxy(ts[i],thr1[i]);

//       lnsrsSeries12.addxy(ts[i],durasi[i]);
//       lnsrsSeries4.addxy(ts[i],(RPAc[i]));
//       lnsrsSeries11.addxy(ts[i],(RPKal[i]));
       lnsrsSeries4.addxy(ts[i],(abs(RPac[i])));
       lnsrsSeries11.addxy(ts[i],(abs(RPkal[i])));
//       lnsrsSeries4.addxy(abs(RPAc[i]),acc1x[i]);
//       lnsrsSeries11.addxy(abs(RPAc[i]),acc1z[i]);
       lnsrsSeries8.addxy(ts[i],velocity[i]);

//       lnsrsSeries14.addxy(ts[i],miup_low[i]);
//       lnsrsSeries15.addxy(ts[i],miup_high[i]);
//       lnsrsSeries30.addxy(ts[i],miud_s[i]);
//       lnsrsSeries31.addxy(ts[i],miud_m[i]);
//       lnsrsSeries32.addxy(ts[i],miud_l[i]);
//       lnsrsSeries16.addxy(ts[i],miud_vl[i]);
//       lnsrsSeries33.addxy(ts[i],miuk_low[i]);
//       lnsrsSeries34.addxy(ts[i],miuk_high[i]);
//       lnsrsSeries17.addxy(ts[i],max1[i]);
//       lnsrsSeries18.addxy(ts[i],max2[i]);
//       lnsrsSeries19.addxy(ts[i],max3[i]);
//       lnsrsSeries35.addxy(ts[i],max4[i]);
       lnsrsSeries40.addxy(ts[i],outf[i]);
       lnsrsSeries41.addxy(ts[i],durasi1[i]);


      if i mod 10 = 0 then
      begin
        //---------------Plot Sheet 1 dan 2---------------//

        lnsrsSeries1.addxy(ts[i],acc1x[i]);
        lnsrsSeries2.addxy(ts[i],acc1z[i]);
        lnsrsSeries3.addxy(ts[i],gy1y[i]);
        lnsrsSeries9.addxy(ts[i],ecg[i]);
        lnsrsSeries5.addxy(ts[i],VRPx[i]);
        lnsrsSeries6.addxy(ts[i],VRPz[i]);
        lnsrsSeries7.addxy(ts[i],VgyroRP[i]);
        lnsrsSeries13.addxy(ts[i],Vecg[i]);



//        Series10.addxy(ts[i],acc2x[i]);
//        Series11.addxy(ts[i],acc2z[i]);
//        Series12.addxy(ts[i],gy2y[i]);
//        Series15.addxy(ts[i],acc3x[i]);
//        Series16.addxy(ts[i],acc3z[i]);
//        Series29.addxy(ts[i],gy3y[i]);
//        Series19.addxy(ts[i],acc4x[i]);
//        Series20.addxy(ts[i],acc4z[i]);
//        Series30.addxy(ts[i],gy4y[i]);

//        Series7.addxy(ts[i],VRLx[i]);
//        Series8.addxy(ts[i],VRLz[i]);
//        Series9.addxy(ts[i],VgyroRL[i]);
//        Series13.addxy(ts[i],VDPAx[i]);
//        Series14.addxy(ts[i],VDPAz[i]);
//        Series27.addxy(ts[i],VgyroDPA[i]);
//        Series17.addxy(ts[i],VDLx[i]);
//        Series18.addxy(ts[i],VDLz[i]);
//        Series28.addxy(ts[i],VgyroDL[i]);

        //---------------Plot Sheet 4---------------//
//        Series33.addxy(ts[i],abs(RUKal[i]));
//        Series36.addxy(ts[i],abs(DPKal[i]));
//        Series31.addxy(ts[i],RUAc[i]);
//        Series34.addxy(ts[i],DPAc[i]);

        //---------------Plot Sheet 3---------------//
//        Series21.addxy(ts[i],abs(RPAc[i]));
//        Series23.addxy(ts[i],abs(RPKal[i]));
//Series22.addxy(ts[i],GyroRP[i]);
//        Series24.addxy(ts[i],abs(RLAc[i]));
//        Series26.addxy(ts[i],abs(RLKal[i]));
//        Series57.addxy(ts[i],abs(DPAAc[i]));
//        Series59.addxy(ts[i],abs(DPAKal[i]));
//        Series60.addxy(ts[i],abs(DLAc[i]));
//        Series62.addxy(ts[i],abs(DLKal[i]));
        //Series22.addxy(ts[i],GyroRP[i]);
        //Series25.addxy(ts[i],GyroRL[i]);
        //Series58.addxy(ts[i],GyroDPA[i]);
        //Series61.addxy(ts[i],GyroDL[i]);

        //---------------Plot Target Sheet 4---------------//
//        Series32.addxy(ts[i],TargetRUC[i]);
//        Series35.addxy(ts[i],TargetDPC[i]);
//
//
//        Series37.addxy(ts[i],VStimRU[i]);
//        Series38.addxy(ts[i],VStimDP[i]);
      end;

       //lnsrsSeries29.addxy(ts[i],rr[i]);
    end;
    t     := t+dt;
    t1    :=t1+dt;
    i     := i+1;
    ndata := i;
    if t1>=tmax then
    begin
      atur_chart;
      t1:=0;
    end;
  end;
end;
procedure TForm1.btn4Click(Sender: TObject);
begin
    t:=t+tmax;
    atur_chart;
end;

procedure TForm1.btn5Click(Sender: TObject);
begin
    t:=0;
    atur_chart;
end;

procedure TForm1.btn6Click(Sender: TObject);
begin
  cmprt1.ShowSetupDialog;
end;

procedure TForm1.btn7Click(Sender: TObject);
begin
if dlgSave1.Execute then
begin
  mmo1.lines.savetofile(dlgSave1.filename+' .txt');
end;
end;

end.

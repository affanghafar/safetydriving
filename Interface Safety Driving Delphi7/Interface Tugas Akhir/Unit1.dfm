object Form1: TForm1
  Left = -9
  Top = 149
  Width = 1535
  Height = 821
  Caption = 'Form1'
  Color = clTeal
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  OnCreate = FormCreate
  PixelsPerInch = 96
  TextHeight = 13
  object lbl3: TLabel
    Left = 552
    Top = 64
    Width = 459
    Height = 29
    Caption = 'Afan Ghafar Al Hadad (07311940000015)'
    Font.Charset = ANSI_CHARSET
    Font.Color = clWhite
    Font.Height = -24
    Font.Name = 'Trebuchet MS'
    Font.Style = [fsBold]
    ParentFont = False
  end
  object lbl4: TLabel
    Left = 600
    Top = 24
    Width = 335
    Height = 37
    Caption = 'INTERFACE TUGAS AKHIR'
    Font.Charset = ANSI_CHARSET
    Font.Color = clWhite
    Font.Height = -29
    Font.Name = 'Trebuchet MS'
    Font.Style = [fsBold]
    ParentFont = False
  end
  object pgc1: TPageControl
    Left = 32
    Top = 128
    Width = 1497
    Height = 609
    ActivePage = ts6
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 0
    object ts1: TTabSheet
      Caption = 'Data Sensor'
      Font.Charset = DEFAULT_CHARSET
      Font.Color = clWindowText
      Font.Height = -11
      Font.Name = 'Tahoma'
      Font.Style = [fsBold]
      ParentFont = False
      object cht1: TChart
        Left = 32
        Top = 32
        Width = 705
        Height = 250
        Legend.Alignment = laBottom
        Legend.CheckBoxes = True
        Legend.TopPos = 4
        Title.Text.Strings = (
          'ADC Sinyal Postural ')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Time (s)'
        LeftAxis.ExactDateTime = False
        LeftAxis.Title.Caption = 'LSB'
        View3D = False
        TabOrder = 0
        PrintMargins = (
          15
          35
          15
          35)
        object lnsrsSeries1: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'Acc_X'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries2: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Title = 'Acc_Z'
          LinePen.Color = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries3: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clBlue
          Title = 'Gyro'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
      object cht2: TChart
        Left = 760
        Top = 24
        Width = 705
        Height = 250
        Legend.Alignment = laBottom
        Legend.CheckBoxes = True
        Legend.TopPos = 4
        Title.Text.Strings = (
          'V Out Sinyal Postural')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Time (s)'
        LeftAxis.Title.Caption = 'Voltage (V)'
        View3D = False
        TabOrder = 1
        object lnsrsSeries5: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'V Out X'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries6: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Title = 'V Out Z'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries7: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clBlue
          Title = 'V Out Gyro'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
      object cht3: TChart
        Left = 32
        Top = 298
        Width = 705
        Height = 250
        Legend.Alignment = laBottom
        Legend.CheckBoxes = True
        Legend.LegendStyle = lsSeries
        Legend.TextStyle = ltsPlain
        Legend.TopPos = 3
        Title.Text.Strings = (
          'ADC Sinyal ECG')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Time (s)'
        LeftAxis.Title.Caption = 'LSB'
        View3D = False
        TabOrder = 2
        PrintMargins = (
          15
          34
          15
          34)
        object lnsrsSeries9: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'Sinyal ECG'
          LinePen.Color = clRed
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
      object cht6: TChart
        Left = 760
        Top = 296
        Width = 705
        Height = 250
        Legend.Alignment = laBottom
        Legend.CheckBoxes = True
        Title.Text.Strings = (
          'V Out Sinyal ECG')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Time (s)'
        LeftAxis.Title.Caption = 'Voltage (V)'
        View3D = False
        TabOrder = 3
        object lnsrsSeries13: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'V Out ECG'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
    end
    object ts2: TTabSheet
      Caption = 'Tilt Segment'
      ImageIndex = 1
      object cht5: TChart
        Left = 8
        Top = 24
        Width = 1137
        Height = 305
        Legend.Alignment = laBottom
        Legend.CheckBoxes = True
        Legend.TopPos = 3
        Title.Text.Strings = (
          'Postural Changes')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Time (s)'
        LeftAxis.Title.Caption = 'Angle (Degree)'
        View3D = False
        TabOrder = 0
        object lnsrsSeries4: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'Tilt Acc'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries8: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Title = 'Kecepatan'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries11: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clBlue
          Title = 'Tilt Kalman'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries12: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 8388672
          Title = 'Durasi'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
    end
    object ts4: TTabSheet
      Caption = 'Pan Tompkins'
      ImageIndex = 3
      object cht4: TChart
        Left = 40
        Top = 64
        Width = 1129
        Height = 321
        Legend.Alignment = laBottom
        Legend.CheckBoxes = True
        Legend.LegendStyle = lsSeries
        Legend.TopPos = 4
        Title.Text.Strings = (
          'V Out Sinyal ECG')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Time (s)'
        LeftAxis.Title.Caption = 'Voltage (V)'
        View3D = False
        TabOrder = 0
        PrintMargins = (
          15
          36
          15
          36)
        object lnsrsSeries10: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'V Out Sinyal ECG'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
          Data = {
            00190000000000000000408A400000000000D08A400000000000288740000000
            0000608540000000000048854000000000000087400000000000A08440000000
            000020844000000000005880400000000000107C400000000000C07540000000
            00004076400000000000A079400000000000A077400000000000907740000000
            00006075400000000000207B4000000000007073400000000000307A40000000
            00000077400000000000A07C400000000000F077400000000000E07640000000
            00000073400000000000107840}
        end
        object lnsrsSeries20: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'Raw Sinyal ECG'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries21: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clNavy
          Title = 'Deteksi QRS'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries22: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clBlue
          Title = 'HR'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries23: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clMaroon
          Title = 'LPF'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries24: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clTeal
          Title = 'HPF'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries25: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGray
          Title = 'Derivative'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries26: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 4210688
          Title = 'Squaring'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries27: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clPurple
          Title = 'MAV'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries28: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 12615680
          Title = 'MAV1'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries29: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clFuchsia
          Title = 'Threshold'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries41: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
    end
    object ts5: TTabSheet
      Caption = 'Fuzzy'
      ImageIndex = 4
      object cht7: TChart
        Left = 16
        Top = 304
        Width = 633
        Height = 257
        Legend.CheckBoxes = True
        Title.Text.Strings = (
          'IMF Durasi Anggukan')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'BPM'
        LeftAxis.Title.Caption = 'Miu'
        View3D = False
        TabOrder = 0
        object lnsrsSeries30: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries31: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries32: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 8404992
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries16: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 64
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
      object cht8: TChart
        Left = 16
        Top = 24
        Width = 633
        Height = 257
        Legend.CheckBoxes = True
        Title.Text.Strings = (
          'IMF Perubahan Sudut')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Derajat (Degree)'
        LeftAxis.Title.Caption = 'Miu'
        View3D = False
        TabOrder = 1
        object lnsrsSeries14: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries15: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
      object cht9: TChart
        Left = 664
        Top = 24
        Width = 641
        Height = 257
        Legend.CheckBoxes = True
        Title.Text.Strings = (
          'IMF Kecepatan Perubahan Sudut')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        BottomAxis.Title.Caption = 'Derajat/Detik'
        LeftAxis.Title.Caption = 'Miu'
        View3D = False
        TabOrder = 2
        object lnsrsSeries33: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries34: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
    end
    object ts6: TTabSheet
      Caption = 'Data'
      ImageIndex = 5
      object lbl1: TLabel
        Left = 24
        Top = 16
        Width = 590
        Height = 13
        Caption = 
          'N Data       Acc1x       Acc1z        Gy1y           ECG        ' +
          '    Tilt         Kalman   Velo           Vecg        QRS    Hear' +
          't Rate'
      end
      object mmo1: TMemo
        Left = 24
        Top = 32
        Width = 1121
        Height = 553
        ScrollBars = ssBoth
        TabOrder = 0
      end
    end
    object ts3: TTabSheet
      Caption = 'Fuzzy 2'
      ImageIndex = 5
      object cht10: TChart
        Left = 16
        Top = 56
        Width = 1289
        Height = 273
        Legend.Visible = False
        Title.Text.Strings = (
          'Output Defuzifikasi Drowsiness Detection')
        BottomAxis.Automatic = False
        BottomAxis.AutomaticMaximum = False
        BottomAxis.AutomaticMinimum = False
        BottomAxis.ExactDateTime = False
        BottomAxis.Increment = 1.000000000000000000
        BottomAxis.Maximum = 7.000000000000000000
        View3D = False
        TabOrder = 0
        object lnsrsSeries17: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Title = 'Output Defuzifikasi Drowsiness Detection'
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries18: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries19: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 64
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries35: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clBlue
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries40: TLineSeries
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 16384
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
      object edt5: TEdit
        Left = 624
        Top = 16
        Width = 129
        Height = 33
        Font.Charset = DEFAULT_CHARSET
        Font.Color = clWindowText
        Font.Height = -21
        Font.Name = 'Tahoma'
        Font.Style = []
        ParentFont = False
        TabOrder = 1
        Text = 'edt5'
      end
    end
    object ts7: TTabSheet
      Caption = 'ts7'
      ImageIndex = 6
      object cht11: TChart
        Left = 32
        Top = 16
        Width = 777
        Height = 297
        Title.Text.Strings = (
          'TChart')
        View3D = False
        TabOrder = 0
        object lnsrsSeries36: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clRed
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries37: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clGreen
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries38: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = 64
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
        object lnsrsSeries39: TLineSeries
          Active = False
          Marks.ArrowLength = 8
          Marks.Callout.Brush.Color = clBlack
          Marks.Callout.Length = 8
          Marks.Visible = False
          SeriesColor = clBlue
          Pointer.InflateMargins = True
          Pointer.Style = psRectangle
          Pointer.Visible = False
          XValues.Name = 'X'
          XValues.Order = loAscending
          YValues.Name = 'Y'
        end
      end
    end
  end
  object btn1: TButton
    Left = 64
    Top = 56
    Width = 75
    Height = 25
    Caption = 'Open Port'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 1
    OnClick = btn1Click
  end
  object btn2: TButton
    Left = 144
    Top = 56
    Width = 75
    Height = 25
    Caption = 'Start'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 2
    OnClick = btn2Click
  end
  object btn3: TButton
    Left = 144
    Top = 88
    Width = 75
    Height = 25
    Caption = 'Close'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 3
    OnClick = btn3Click
  end
  object edt1: TEdit
    Left = 224
    Top = 88
    Width = 89
    Height = 21
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 4
    Text = 'edt1'
  end
  object btn6: TButton
    Left = 64
    Top = 24
    Width = 75
    Height = 25
    Caption = 'Port Setting'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 5
    OnClick = btn6Click
  end
  object btn7: TButton
    Left = 64
    Top = 88
    Width = 75
    Height = 25
    Caption = 'Save Data'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -11
    Font.Name = 'Tahoma'
    Font.Style = [fsBold]
    ParentFont = False
    TabOrder = 6
    OnClick = btn7Click
  end
  object edt2: TEdit
    Left = 320
    Top = 88
    Width = 81
    Height = 21
    TabOrder = 7
    Text = 'edt2'
  end
  object edt3: TEdit
    Left = 224
    Top = 56
    Width = 89
    Height = 21
    TabOrder = 8
    Text = 'edt3'
  end
  object edt4: TEdit
    Left = 320
    Top = 56
    Width = 81
    Height = 21
    TabOrder = 9
    Text = 'edt4'
  end
  object edt6: TEdit
    Left = 408
    Top = 88
    Width = 129
    Height = 33
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    TabOrder = 10
    Text = 'edt5'
  end
  object cmprt1: TComPort
    BaudRate = br115200
    Port = 'COM6'
    Parity.Bits = prNone
    StopBits = sbOneStopBit
    DataBits = dbEight
    EventChar = 'e'
    Events = [evRxChar, evTxEmpty, evRxFlag, evRing, evBreak, evCTS, evDSR, evError, evRLSD, evRx80Full]
    FlowControl.OutCTSFlow = False
    FlowControl.OutDSRFlow = False
    FlowControl.ControlDTR = dtrDisable
    FlowControl.ControlRTS = rtsDisable
    FlowControl.XonXoffOut = False
    FlowControl.XonXoffIn = False
    StoredProps = [spBasic]
    TriggersOnRxChar = False
    Left = 24
    Top = 16
  end
  object cmdtpckt1: TComDataPacket
    ComPort = cmprt1
    IncludeStrings = True
    StartString = 'e'
    Size = 24
    OnPacket = DP1
    Left = 24
    Top = 48
  end
  object dlgSave1: TSaveDialog
    Left = 24
    Top = 80
  end
end

within ;
package OCRAIOSPSSE
  package Components

    model PowerToReal
      OpenIPSL.Interfaces.PwPin In
        annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
            iconTransformation(extent={{-126,-14},{-100,12}})));
      Modelica.Blocks.Interfaces.RealOutput Out
        annotation (Placement(transformation(extent={{100,-8},{120,12}}),
            iconTransformation(extent={{100,-12},{124,12}})));
        Real Vmag;
         Real Vang;
        Real Imag;
        Real Iang;
    equation
        Vmag = sqrt((In.vr)^2+(In.vi)^2);
      Vang = atan(In.vi/(max(In.vr,0.0001)));
        Out = sqrt((In.ir)^2+(In.ii)^2);
        Iang = atan(In.ii/(max(In.ir,0.0001)));
        Out = Imag;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end PowerToReal;

    model PSSEGeneratorTGOV
      OpenIPSL.Electrical.Machines.PSSE.GENSAL gENSAL(
        Tpd0=5,
        Tppd0=0.07,
        Tppq0=0.09,
        H=4.28,
        D=0,
        Xd=2.5,
        Xq=1.65,
        Xpd=0.76,
        Xppd=0.62,
        Xppq=0.58,
        Xl=0.066,
        S10=0.11,
        S12=0.39,
        R_a=0.01,
        M_b=M_b,
        V_b=V_b,
        V_0=V_0,
        angle_0=angle_0,
        P_0=P_0,
        Q_0=Q_0)  annotation (Placement(transformation(extent={{42,-10},{62,10}})));
      OpenIPSL.Interfaces.PwPin pwPin
        annotation (Placement(transformation(extent={{94,-12},{114,8}}),
            iconTransformation(extent={{94,-12},{114,8}})));
      OpenIPSL.Electrical.Controls.PSSE.ES.ESST1A eSST1A(
        T_R=0.04,
        V_IMAX=999,
        V_IMIN=-999,
        T_C=1,
        T_B=10,
        T_C1=0,
        T_B1=0,
        K_A=190,
        T_A=0.001,
        V_AMAX=999,
        V_AMIN=-999,
        V_RMAX=7.8,
        V_RMIN=-6.7,
        K_C=0.08,
        K_F=1,
        T_F=0.001,
        K_LR=0,
        I_LR=0) annotation (Placement(transformation(extent={{-34,-44},{6,-8}})));
      Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-62,-90})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
      Modelica.Blocks.Sources.Constant const2(k=-Modelica.Constants.inf)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={2,-88})));
      Modelica.Blocks.Sources.Constant const3(k=0)
                                              annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-84,10})));
      parameter OpenIPSL.Types.VoltageKilo V_b=400 "Base voltage of the bus";
      parameter Modelica.SIunits.PerUnit V_0=1 "Voltage magnitude (pu)";
      parameter Modelica.SIunits.Conversions.NonSIunits.Angle_deg angle_0=0
        "Voltage angle";
      parameter OpenIPSL.Types.ActivePowerMega P_0=1 "Active power";
      parameter OpenIPSL.Types.ReactivePowerMega Q_0=0 "Reactive power";
      parameter Real M_b=460 "Machine base power (MVA)";
      OpenIPSL.Electrical.Controls.PSSE.OEL.OEL oEL(
        IFD1=3.78,
        IFD2=4.5,
        IFD3=5.76,
        TIME1=60,
        TIME2=30,
        TIME3=15) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-30,-88})));
      OpenIPSL.Electrical.Controls.PSSE.TG.TGOV1 tGOV1_2(
        R=0.05,
        D_t=0.2,
        V_MIN=0,
        T_1=5,
        T_2=0.05,
        T_3=0.5,
        V_MAX=0.7)
        annotation (Placement(transformation(extent={{2,2},{20,16}})));
    equation
      connect(eSST1A.EFD, gENSAL.EFD) annotation (Line(points={{7,-24},{24,-24},{24,
              -5},{40,-5}}, color={0,0,127}));
      connect(const.y, eSST1A.VUEL) annotation (Line(points={{-62,-79},{-62,-68},{-26,
              -68},{-26,-44},{-27,-44}}, color={0,0,127}));
      connect(eSST1A.VOTHSG2, const3.y) annotation (Line(points={{-34,-11},{-54,-11},
              {-54,10},{-73,10}}, color={0,0,127}));
      connect(eSST1A.VOTHSG, const3.y) annotation (Line(points={{-34,-15},{-42,-15},
              {-42,-16},{-54,-16},{-54,10},{-73,10}}, color={0,0,127}));
      connect(gENSAL.XADIFD, eSST1A.XADIFD) annotation (Line(points={{62.8,-9},{76,-9},
              {76,-58},{-5,-58},{-5,-43.6}}, color={0,0,127}));
      connect(gENSAL.p, pwPin)
        annotation (Line(points={{62,0},{84,0},{84,-2},{104,-2}},
                                                  color={0,0,255}));
      connect(const2.y, eSST1A.VUEL2) annotation (Line(points={{2,-77},{2,-60},
              {-14.99,-60},{-14.99,-43.99}},
                                     color={0,0,127}));
      connect(eSST1A.VUEL3, eSST1A.VUEL2) annotation (Line(points={{-9.015,-43.995},
              {-8,-60},{-14.99,-60},{-14.99,-43.99}}, color={0,0,127}));
      connect(gENSAL.ETERM, eSST1A.VT) annotation (Line(points={{63,-3},{82,-3},{82,
              -108},{-80,-108},{-80,-19.025},{-33.975,-19.025}}, color={0,0,127}));
      connect(eSST1A.ECOMP, eSST1A.VT) annotation (Line(points={{-34,-24},{-56,-24},
              {-56,-20},{-33.975,-19.025}}, color={0,0,127}));
      connect(eSST1A.EFD0, gENSAL.EFD0) annotation (Line(points={{-34,-37},{-40,-37},
              {-40,-62},{78,-62},{78,-5},{63,-5}}, color={0,0,127}));
      connect(oEL.VOEL, eSST1A.VOEL) annotation (Line(points={{-30,-77.5},{
              -30,-74},{-21,-74},{-21,-44}}, color={0,0,127}));
      connect(oEL.IFD, gENSAL.EFD0) annotation (Line(points={{-30,-98.5},{-30,
              -104},{58,-104},{58,-62},{78,-62},{78,-5},{63,-5}}, color={0,0,
              127}));
      connect(gENSAL.PMECH, tGOV1_2.PMECH) annotation (Line(points={{40,5},{
              34,5},{34,9.36842},{20.375,9.36842}}, color={0,0,127}));
      connect(gENSAL.PMECH0, tGOV1_2.PMECH0) annotation (Line(points={{63,5},
              {68,5},{68,26},{-18,26},{-18,4},{2.75,4},{2.75,4.94737}}, color=
             {0,0,127}));
      connect(tGOV1_2.SPEED, gENSAL.SPEED) annotation (Line(points={{2.75,
              13.0526},{-2,13.0526},{-2,22},{66,22},{66,7},{63,7}}, color={0,
              0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-140},{100,100}}), graphics={
                                       Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid), Line(points={{-78,-22},{-20,
                  38},{46,-16},{82,48},{82,50}}, color={28,108,200})}),
                                                                     Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{
                100,100}})));
    end PSSEGeneratorTGOV;

    package RelayPack
      package Components
        model ExtractingTimeOfFault
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Interfaces.RealInput u
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Logical.Switch switch1
            annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
          Modelica.Blocks.Math.Product product
            annotation (Placement(transformation(extent={{40,-4},{60,16}})));
          Modelica.Blocks.Sources.Constant const(k=1000)
            annotation (Placement(transformation(extent={{-76,32},{-56,52}})));
          Modelica.Blocks.Sources.Constant const1(k=1)
            annotation (Placement(transformation(extent={{-78,-54},{-58,-34}})));
          Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold=
               0.9)
            annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
          Modelica.Blocks.Continuous.LimIntegrator limIntegrator(outMax=1)
            annotation (Placement(transformation(extent={{-32,30},{-12,50}})));
        equation
          connect(product.y, y) annotation (Line(points={{61,6},{76,6},{76,0},{
                  110,0}},
                       color={0,0,127}));
          connect(u, product.u1) annotation (Line(points={{-120,0},{-86,0},{-86,
                  74},{16,74},{16,12},{38,12}},
                                           color={0,0,127}));
          connect(switch1.y, product.u2) annotation (Line(points={{17,0},{38,0}},
                                color={0,0,127}));
          connect(u,greaterEqualThreshold. u)
            annotation (Line(points={{-120,0},{-64,0}},   color={0,0,127}));
          connect(greaterEqualThreshold.y, switch1.u2) annotation (Line(points={{-41,0},
                  {-6,0}},                         color={255,0,255}));
          connect(const1.y, switch1.u3) annotation (Line(points={{-57,-44},{-32,
                  -44},{-32,-8},{-6,-8}}, color={0,0,127}));
          connect(const.y, limIntegrator.u) annotation (Line(points={{-55,42},{
                  -44,42},{-44,40},{-34,40}}, color={0,0,127}));
          connect(limIntegrator.y, switch1.u1) annotation (Line(points={{-11,40},
                  {-2,40},{-2,16},{-14,16},{-14,8},{-6,8}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                         Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end ExtractingTimeOfFault;

        model CalculatingOperationTime
          Modelica.Blocks.Interfaces.RealInput Control
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
          Modelica.Blocks.Interfaces.RealInput CurrentInput
            annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
          Modelica.Blocks.Interfaces.RealOutput OperationTime
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Logical.Switch switch1
            annotation (Placement(transformation(extent={{-12,44},{8,64}})));
          Modelica.Blocks.Sources.Constant Const(k=C)
            annotation (Placement(transformation(extent={{-58,74},{-38,94}})));
          Modelica.Blocks.Sources.Constant const1(k=0)
            annotation (Placement(transformation(extent={{-82,-4},{-62,16}})));
          Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold=
               0)
            annotation (Placement(transformation(extent={{-82,26},{-62,46}})));
          Modelica.Blocks.Math.Division division1
            annotation (Placement(transformation(extent={{-62,-58},{-42,-38}})));
          Modelica.Blocks.Sources.Constant pickcupcurrent(k=ls)
            annotation (Placement(transformation(extent={{-90,-90},{-70,-70}})));
          Modelica.Blocks.Sources.Constant const2(k=1)
            annotation (Placement(transformation(extent={{-34,-92},{-14,-72}})));
          Modelica.Blocks.Math.Add add(k2=-1)
            annotation (Placement(transformation(extent={{12,-68},{32,-48}})));
          Modelica.Blocks.Math.Product product
            annotation (Placement(transformation(extent={{58,28},{78,48}})));
          Modelica.Blocks.Sources.Constant TimeMultiplierSetting(k=
                0.439238653001)
            annotation (Placement(transformation(extent={{-10,4},{10,24}})));
          Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold
            annotation (Placement(transformation(extent={{-82,50},{-62,70}})));
          toThePower toThePower1
            annotation (Placement(transformation(extent={{-22,-62},{-2,-42}})));
          Modelica.Blocks.Sources.Constant const(k=alpha)
            annotation (Placement(transformation(extent={{-62,-90},{-42,-70}})));
          Modelica.Blocks.Logical.Nand nand
            annotation (Placement(transformation(extent={{-54,44},{-34,64}})));
          parameter Real alpha=0.02 "Constant output value";
          parameter Real TMS=0.5 "Constant output value";
          parameter Real C=0.14 "Constant output value";
          parameter Real ls=1 "Constant output value";
          Modelica.Blocks.Sources.Constant eps(k=0.41)
            annotation (Placement(transformation(extent={{34,-94},{54,-74}})));
          Y3 y3_1 annotation (Placement(transformation(extent={{58,-54},{78,-34}})));
          Modelica.Blocks.Math.Product product1
            annotation (Placement(transformation(extent={{24,40},{44,60}})));
          parameter Real k=eps "Constant output value";
        equation
          connect(CurrentInput, division1.u1) annotation (Line(points={{-120,-60},{
                  -90,-60},{-90,-42},{-64,-42}}, color={0,0,127}));
          connect(pickcupcurrent.y, division1.u2) annotation (Line(points={{-69,-80},
                  {-66,-80},{-66,-54},{-64,-54}}, color={0,0,127}));
          connect(product.y, OperationTime) annotation (Line(points={{79,38},{82,38},
                  {82,0},{110,0}}, color={0,0,127}));
          connect(add.u2, const2.y) annotation (Line(points={{10,-64},{0,-64},{0,
                  -82},{-13,-82}}, color={0,0,127}));
          connect(TimeMultiplierSetting.y, product.u2) annotation (Line(points={{11,14},
                  {14,14},{14,32},{56,32}},
                           color={0,0,127}));
          connect(Control, lessEqualThreshold.u)
            annotation (Line(points={{-120,60},{-84,60}}, color={0,0,127}));
          connect(greaterEqualThreshold.u, lessEqualThreshold.u) annotation (Line(
                points={{-84,36},{-88,36},{-88,60},{-84,60}}, color={0,0,127}));
          connect(Const.y, switch1.u1) annotation (Line(points={{-37,84},{-22,
                  84},{-22,62},{-14,62}},
                             color={0,0,127}));
          connect(const1.y, switch1.u3) annotation (Line(points={{-61,6},{-26,6},
                  {-26,46},{-14,46}}, color={0,0,127}));
          connect(division1.y, toThePower1.u) annotation (Line(points={{-41,-48},{
                  -30,-48},{-30,-46},{-24,-46}},
                                             color={0,0,127}));
          connect(toThePower1.y, add.u1) annotation (Line(points={{-1,-52},{10,-52}},
                                  color={0,0,127}));
          connect(const.y, toThePower1.alpha) annotation (Line(points={{-41,-80},{
                  -40,-80},{-40,-64},{-28,-64},{-28,-56},{-24,-56}},
                                                                 color={0,0,127}));
          connect(lessEqualThreshold.y, nand.u1) annotation (Line(points={{-61,60},{
                  -61,57},{-56,57},{-56,54}}, color={255,0,255}));
          connect(greaterEqualThreshold.y, nand.u2) annotation (Line(points={{-61,36},
                  {-58,36},{-58,46},{-56,46}}, color={255,0,255}));
          connect(nand.y, switch1.u2)
            annotation (Line(points={{-33,54},{-14,54}}, color={255,0,255}));
          connect(add.y, y3_1.x) annotation (Line(points={{33,-58},{44,-58},{44,-38},
                  {56,-38}}, color={0,0,127}));
          connect(eps.y, y3_1.eps) annotation (Line(points={{55,-84},{76,-84},{
                  76,-62},{50,-62},{50,-48},{56,-48}},
                                                    color={0,0,127}));
          connect(product1.u1, switch1.y) annotation (Line(points={{22,56},{16,
                  56},{16,54},{9,54}},
                                   color={0,0,127}));
          connect(product1.y, product.u1) annotation (Line(points={{45,50},{50.5,50},
                  {50.5,44},{56,44}}, color={0,0,127}));
          connect(y3_1.y, product1.u2) annotation (Line(points={{79,-44},{88,
                  -44},{88,-14},{-18,-14},{-18,38},{16,38},{16,44},{22,44}},
                color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                         Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end CalculatingOperationTime;

        model Timer
          Modelica.Blocks.Sources.Clock clock
            annotation (Placement(transformation(extent={{-96,-6},{-76,14}})));
          Modelica.Blocks.Math.Add add
            annotation (Placement(transformation(extent={{44,-10},{64,10}})));
          Modelica.Blocks.Sources.Constant const(k=-1)
            annotation (Placement(transformation(extent={{-96,-36},{-76,-16}})));
          Modelica.Blocks.Math.Product product
            annotation (Placement(transformation(extent={{-48,-24},{-28,-4}})));
          Modelica.Blocks.Sources.Clock clock1
            annotation (Placement(transformation(extent={{-4,-62},{16,-42}})));
          Modelica.Blocks.Interfaces.RealInput u
            annotation (Placement(transformation(extent={{-140,22},{-100,62}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold=
               0.90)
            annotation (Placement(transformation(extent={{-66,32},{-46,52}})));
          Modelica.Blocks.Logical.Switch switch1
            annotation (Placement(transformation(extent={{-12,26},{8,46}})));
          Modelica.Blocks.Sources.Constant const1(k=0)
            annotation (Placement(transformation(extent={{-92,72},{-72,92}})));
        equation
          connect(clock1.y, add.u2) annotation (Line(points={{17,-52},{28,-52},{28,-6},
                  {42,-6}}, color={0,0,127}));
          connect(u, greaterEqualThreshold.u)
            annotation (Line(points={{-120,42},{-68,42}}, color={0,0,127}));
          connect(switch1.y, add.u1) annotation (Line(points={{9,36},{28,36},{28,6},{
                  42,6}}, color={0,0,127}));
          connect(greaterEqualThreshold.y, switch1.u2) annotation (Line(points={{-45,
                  42},{-24,42},{-24,36},{-14,36}}, color={255,0,255}));
          connect(const.y, product.u2) annotation (Line(points={{-75,-26},{-62,-26},{
                  -62,-20},{-50,-20}}, color={0,0,127}));
          connect(clock.y, product.u1) annotation (Line(points={{-75,4},{-62,4},{-62,
                  -8},{-50,-8}}, color={0,0,127}));
          connect(product.y, switch1.u3) annotation (Line(points={{-27,-14},{-22,-14},
                  {-22,28},{-14,28}}, color={0,0,127}));
          connect(const1.y, switch1.u1) annotation (Line(points={{-71,82},{-18,82},{
                  -18,44},{-14,44}}, color={0,0,127}));
          connect(add.y, y)
            annotation (Line(points={{65,0},{110,0}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                         Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end Timer;

        package DiscreteFourier
          model DiscreteFourierOld
            import RelayModel;
            import RelayModel;
            Modelica.Blocks.Interfaces.RealInput Input
              annotation (Placement(transformation(extent={{-240,60},{-200,100}})));
            Modelica.Blocks.Interfaces.RealOutput Magnitude
              annotation (Placement(transformation(extent={{100,10},{120,30}})));
            Modelica.Blocks.Interfaces.RealOutput Phase
              annotation (Placement(transformation(extent={{100,-30},{120,-10}})));
            Modelica.Blocks.Math.Gain gain(k=2*Modelica.Constants.pi*1*50)
              annotation (Placement(transformation(extent={{-164,-66},{-144,-46}})));
            Modelica.Blocks.Math.Gain gain1(k=2)
              annotation (Placement(transformation(extent={{-34,-80},{-14,-60}})));
            Modelica.Blocks.Math.Sin sin
              annotation (Placement(transformation(extent={{-124,-56},{-104,-36}})));
            Modelica.Blocks.Math.Cos cos
              annotation (Placement(transformation(extent={{-118,-90},{-98,-70}})));
            Modelica.Blocks.Sources.Clock clock
              annotation (Placement(transformation(extent={{-208,-66},{-188,-46}})));
            Modelica.Blocks.Routing.Multiplex2 multiplex2_1
              annotation (Placement(transformation(extent={{-78,-68},{-58,-48}})));
            Modelica.Blocks.Math.Product product
              annotation (Placement(transformation(extent={{-172,64},{-152,84}})));
            Modelica.Blocks.Math.Gain gain2(k=180/Modelica.Constants.pi)
              annotation (Placement(transformation(extent={{66,-30},{86,-10}})));
            Modelica.Blocks.Math.RectangularToPolar rectangularToPolar
              annotation (Placement(transformation(extent={{-12,74},{8,94}})));
            Modelica.Blocks.Interfaces.RealOutput DMVReal
              annotation (Placement(transformation(extent={{-44,102},{-24,122}})));
            Modelica.Blocks.Interfaces.RealOutput DMVImag
              annotation (Placement(transformation(extent={{-42,22},{-22,42}})));
            Modelica.Blocks.Interfaces.RealOutput RealOut
              annotation (Placement(transformation(extent={{38,92},{58,112}})));
            Modelica.Blocks.Interfaces.RealOutput ImagOut
              annotation (Placement(transformation(extent={{14,20},{34,40}})));
            Modelica.Blocks.Interfaces.RealOutput before
              annotation (Placement(transformation(extent={{-114,116},{-92,136}})));
            Components.DiscreteMeanValue RealDMV(Ts=5e-5)
              annotation (Placement(transformation(extent={{-106,78},{-70,100}})));
            Components.DiscreteMeanValue ImagDMV
              annotation (Placement(transformation(extent={{-104,50},{-68,72}})));
          equation
            connect(gain2.y, Phase)
              annotation (Line(points={{87,-20},{110,-20}}, color={0,0,127}));
            connect(Input, product.u1) annotation (Line(points={{-220,80},{-174,80}},
                                    color={0,0,127}));
            connect(gain1.y, product.u2) annotation (Line(points={{-13,-70},{-8,-70},{
                    -8,22},{-188,22},{-188,68},{-174,68}},
                                                        color={0,0,127}));
            connect(clock.y, gain.u) annotation (Line(points={{-187,-56},{-166,-56}},
                                      color={0,0,127}));
            connect(multiplex2_1.y[1], gain1.u) annotation (Line(points={{-57,-58},{-48,
                    -58},{-48,-70},{-36,-70}}, color={0,0,127}));
            connect(rectangularToPolar.y_arg, gain2.u) annotation (Line(points={{9,78},{
                    36,78},{36,-20},{64,-20}}, color={0,0,127}));
            connect(rectangularToPolar.y_abs, Magnitude) annotation (Line(points={{9,90},{
                    56,90},{56,20},{110,20}},  color={0,0,127}));
            connect(sin.y, multiplex2_1.u1[1]) annotation (Line(points={{-103,-46},{-88,
                    -46},{-88,-52},{-80,-52}}, color={0,0,127}));
            connect(cos.y, multiplex2_1.u2[1]) annotation (Line(points={{-97,-80},{-88.5,
                    -80},{-88.5,-64},{-80,-64}}, color={0,0,127}));
            connect(gain.y, sin.u) annotation (Line(points={{-143,-56},{-134,-56},{-134,
                    -46},{-126,-46}}, color={0,0,127}));
            connect(cos.u, sin.u) annotation (Line(points={{-120,-80},{-134,-80},{-134,
                    -46},{-126,-46}}, color={0,0,127}));
            connect(rectangularToPolar.y_arg, ImagOut) annotation (Line(points={{9,78},
                    {12,78},{12,30},{24,30}}, color={0,0,127}));
            connect(rectangularToPolar.y_abs, RealOut) annotation (Line(points={{9,90},
                    {24,90},{24,102},{48,102}}, color={0,0,127}));
            connect(product.y, before) annotation (Line(points={{-151,74},{-130,74},{
                    -130,126},{-103,126}}, color={0,0,127}));
            connect(product.y, RealDMV.u) annotation (Line(points={{-151,74},{-128,
                    74},{-128,88},{-108,88}}, color={0,0,127}));
            connect(product.y, ImagDMV.u) annotation (Line(points={{-151,74},{-128,
                    74},{-128,60},{-106,60}}, color={0,0,127}));
            connect(RealDMV.y, rectangularToPolar.u_re) annotation (Line(points={{
                    -69,88},{-40,88},{-40,90},{-14,90}}, color={0,0,127}));
            connect(ImagDMV.y, rectangularToPolar.u_im) annotation (Line(points={{
                    -67,60},{-40,60},{-40,78},{-14,78}}, color={0,0,127}));
            connect(RealDMV.y, DMVReal) annotation (Line(points={{-69,88},{-56,88},
                    {-56,112},{-34,112}}, color={0,0,127}));
            connect(ImagDMV.y, DMVImag) annotation (Line(points={{-67,60},{-54,60},
                    {-54,32},{-32,32}}, color={0,0,127}));
            annotation (
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{100,
                      120}})),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
                      100,120}})));
          end DiscreteFourierOld;

          package Components
            model DiscreteMeanValue
              Modelica.Blocks.Interfaces.RealInput u
                annotation (Placement(transformation(extent={{-180,-20},{-140,20}})));
              Modelica.Blocks.Interfaces.RealOutput y
                annotation (Placement(transformation(extent={{220,-10},{240,10}})));
              Modelica.Blocks.Discrete.UnitDelay unitDelay(samplePeriod=Ts)
                annotation (Placement(transformation(extent={{-88,-88},{-68,-68}})));
              Modelica.Blocks.Math.Gain gain(k=0)
                annotation (Placement(transformation(extent={{-58,-88},{-38,-68}})));
              Modelica.Blocks.Math.Gain gain1(k=0)
                annotation (Placement(transformation(extent={{-54,-48},{-34,-28}})));
              Modelica.Blocks.Math.Add add(k2=-1)
                annotation (Placement(transformation(extent={{-24,-84},{-4,-64}})));
              Modelica.Blocks.Math.Add add1
                annotation (Placement(transformation(extent={{86,-8},{106,12}})));
              Modelica.Blocks.Logical.Switch switch1
                annotation (Placement(transformation(extent={{158,-30},{178,-10}})));
              Modelica.Blocks.Sources.Constant const(k=FreqHz)
                annotation (Placement(transformation(extent={{-24,4},{-4,24}})));
              Modelica.Blocks.Sources.Constant const1(k=Vinit)
                annotation (Placement(transformation(extent={{98,-40},{118,-20}})));
              Modelica.Blocks.Math.Add add2(k1=-1)
                annotation (Placement(transformation(extent={{-30,38},{-10,58}})));
              Modelica.Blocks.Sources.BooleanStep booleanStep(                  startValue=
                    false, startTime=1/50)
                           annotation (Placement(transformation(extent={{98,26},{118,46}})));
              Modelica.Blocks.Math.Product product
                annotation (Placement(transformation(extent={{26,24},{46,44}})));
              Modelica.Blocks.Discrete.TransferFunction transferFunction(
                a={2,-2},
                b={Ts,Ts},
                samplePeriod=Ts)
                annotation (Placement(transformation(extent={{-120,52},{-100,72}})));
              Modelica.Blocks.Nonlinear.VariableDelay variableDelay(delayMax=2 + 50e-6)
                annotation (Placement(transformation(extent={{-78,58},{-58,78}})));
              Modelica.Blocks.Sources.Constant const2(k=Delay)
                annotation (Placement(transformation(extent={{-112,24},{-92,44}})));
              parameter Real FreqHz=50 "Constant output value";
              parameter Modelica.SIunits.Time Ts=50e-6 "Sample period of component";
              parameter Real Vinit=1 "Constant output value";
              parameter Real Delay=2 "Constant output value";
            equation
              connect(u, unitDelay.u) annotation (Line(points={{-160,0},{-94,0},{-94,-78},{
                      -90,-78}}, color={0,0,127}));
              connect(gain1.y, add.u1) annotation (Line(points={{-33,-38},{-30,-38},{-30,
                      -68},{-26,-68}}, color={0,0,127}));
              connect(gain.y, add.u2) annotation (Line(points={{-37,-78},{-32,-78},{-32,-80},
                      {-26,-80}}, color={0,0,127}));
              connect(unitDelay.y, gain.u) annotation (Line(points={{-67,-78},{-60,-78}},
                                           color={0,0,127}));
              connect(gain1.u, unitDelay.u) annotation (Line(points={{-56,-38},{-94,-38},{
                      -94,-78},{-90,-78}}, color={0,0,127}));
              connect(add.y, add1.u2) annotation (Line(points={{-3,-74},{10,-74},{10,-4},{
                      84,-4}}, color={0,0,127}));
              connect(switch1.y, y) annotation (Line(points={{179,-20},{182,-20},{182,0},{
                      230,0}}, color={0,0,127}));
              connect(booleanStep.y, switch1.u2) annotation (Line(points={{119,36},{136,36},
                      {136,-20},{156,-20}}, color={255,0,255}));
              connect(const1.y, switch1.u3) annotation (Line(points={{119,-30},{136,-30},{
                      136,-28},{156,-28}}, color={0,0,127}));
              connect(add1.y, switch1.u1) annotation (Line(points={{107,2},{130,2},{130,-12},
                      {156,-12}}, color={0,0,127}));
              connect(product.y, add1.u1)
                annotation (Line(points={{47,34},{62,34},{62,8},{84,8}}, color={0,0,127}));
              connect(const.y, product.u2)
                annotation (Line(points={{-3,14},{6,14},{6,28},{24,28}}, color={0,0,127}));
              connect(transferFunction.u, unitDelay.u) annotation (Line(points={{-122,62},{
                      -124,62},{-124,0},{-94,0},{-94,-78},{-90,-78}}, color={0,0,127}));
              connect(product.u1, add2.y)
                annotation (Line(points={{24,40},{6,40},{6,48},{-9,48}}, color={0,0,127}));
              connect(transferFunction.y, variableDelay.u) annotation (Line(points={{-99,62},
                      {-90,62},{-90,68},{-80,68}}, color={0,0,127}));
              connect(variableDelay.y, add2.u1) annotation (Line(points={{-57,68},{-48,68},
                      {-48,54},{-32,54}}, color={0,0,127}));
              connect(const2.y, variableDelay.delayTime) annotation (Line(points={{-91,34},
                      {-86,34},{-86,62},{-80,62}}, color={0,0,127}));
              connect(transferFunction.y, add2.u2) annotation (Line(points={{-99,62},{-88,
                      62},{-88,42},{-32,42}}, color={0,0,127}));
              annotation (
                Diagram(coordinateSystem(extent={{-140,-100},{220,120}})),
                Icon(coordinateSystem(extent={{-140,-100},{220,120}}), graphics={
                      Rectangle(extent={{-140,120},{220,-98}}, lineColor={28,108,
                          200})}));
            end DiscreteMeanValue;

            model DiscreteMeanValueReal
              Modelica.Blocks.Interfaces.RealInput u
                annotation (Placement(transformation(extent={{-180,-20},{-140,20}})));
              Modelica.Blocks.Interfaces.RealOutput y
                annotation (Placement(transformation(extent={{220,-10},{240,10}})));
              Modelica.Blocks.Discrete.UnitDelay unitDelay(samplePeriod=5e-5)
                annotation (Placement(transformation(extent={{-88,-88},{-68,-68}})));
              Modelica.Blocks.Math.Gain gain(k=2)
                annotation (Placement(transformation(extent={{-58,-88},{-38,-68}})));
              Modelica.Blocks.Math.Gain gain1(k=50)
                annotation (Placement(transformation(extent={{-54,-48},{-34,-28}})));
              Modelica.Blocks.Math.Add add(k2=-1)
                annotation (Placement(transformation(extent={{-24,-84},{-4,-64}})));
              Modelica.Blocks.Math.Add add1
                annotation (Placement(transformation(extent={{86,-8},{106,12}})));
              Modelica.Blocks.Logical.Switch switch1
                annotation (Placement(transformation(extent={{158,-30},{178,-10}})));
              Modelica.Blocks.Sources.Constant const(k=50)
                annotation (Placement(transformation(extent={{-24,4},{-4,24}})));
              Modelica.Blocks.Sources.Constant const1(k=1)
                annotation (Placement(transformation(extent={{98,-40},{118,-20}})));
              Modelica.Blocks.Math.Add add2(k1=-1)
                annotation (Placement(transformation(extent={{-30,38},{-10,58}})));
              Modelica.Blocks.Sources.BooleanStep booleanStep(                  startValue=
                    false, startTime=1/50)
                           annotation (Placement(transformation(extent={{98,26},{118,46}})));
              Modelica.Blocks.Math.Product product
                annotation (Placement(transformation(extent={{26,24},{46,44}})));
              Modelica.Blocks.Discrete.TransferFunction transferFunction(
                a={2,-2},
                b={5e-5,5e-5},
                samplePeriod=5e-5)
                annotation (Placement(transformation(extent={{-120,52},{-100,72}})));
              Modelica.Blocks.Nonlinear.VariableDelay variableDelay(delayMax=2 +
                    5e-5)
                annotation (Placement(transformation(extent={{-72,52},{-52,72}})));
              Modelica.Blocks.Sources.Constant const2(k=2)
                annotation (Placement(transformation(extent={{-112,24},{-92,44}})));
              Modelica.Blocks.Interfaces.RealOutput top
                annotation (Placement(transformation(extent={{62,48},{84,68}})));
              Modelica.Blocks.Interfaces.RealOutput bottom
                annotation (Placement(transformation(extent={{58,-50},{80,-30}})));
            equation
              connect(u, unitDelay.u) annotation (Line(points={{-160,0},{-94,0},{-94,-78},{
                      -90,-78}}, color={0,0,127}));
              connect(gain1.y, add.u1) annotation (Line(points={{-33,-38},{-30,-38},{-30,
                      -68},{-26,-68}}, color={0,0,127}));
              connect(gain.y, add.u2) annotation (Line(points={{-37,-78},{-32,-78},{-32,-80},
                      {-26,-80}}, color={0,0,127}));
              connect(unitDelay.y, gain.u) annotation (Line(points={{-67,-78},{-60,-78}},
                                           color={0,0,127}));
              connect(gain1.u, unitDelay.u) annotation (Line(points={{-56,-38},{-94,-38},{
                      -94,-78},{-90,-78}}, color={0,0,127}));
              connect(add.y, add1.u2) annotation (Line(points={{-3,-74},{10,-74},{10,-4},{
                      84,-4}}, color={0,0,127}));
              connect(switch1.y, y) annotation (Line(points={{179,-20},{182,-20},{182,0},{
                      230,0}}, color={0,0,127}));
              connect(booleanStep.y, switch1.u2) annotation (Line(points={{119,36},{136,36},
                      {136,-20},{156,-20}}, color={255,0,255}));
              connect(const1.y, switch1.u3) annotation (Line(points={{119,-30},{136,-30},{
                      136,-28},{156,-28}}, color={0,0,127}));
              connect(add1.y, switch1.u1) annotation (Line(points={{107,2},{130,2},{130,-12},
                      {156,-12}}, color={0,0,127}));
              connect(product.y, add1.u1)
                annotation (Line(points={{47,34},{62,34},{62,8},{84,8}}, color={0,0,127}));
              connect(const.y, product.u2)
                annotation (Line(points={{-3,14},{6,14},{6,28},{24,28}}, color={0,0,127}));
              connect(transferFunction.u, unitDelay.u) annotation (Line(points={{-122,62},{
                      -124,62},{-124,0},{-94,0},{-94,-78},{-90,-78}}, color={0,0,127}));
              connect(product.u1, add2.y)
                annotation (Line(points={{24,40},{6,40},{6,48},{-9,48}}, color={0,0,127}));
              connect(transferFunction.y, variableDelay.u) annotation (Line(points={{-99,62},
                      {-74,62}},                   color={0,0,127}));
              connect(variableDelay.y, add2.u1) annotation (Line(points={{-51,62},{-48,62},
                      {-48,54},{-32,54}}, color={0,0,127}));
              connect(const2.y, variableDelay.delayTime) annotation (Line(points={{-91,34},
                      {-86,34},{-86,56},{-74,56}}, color={0,0,127}));
              connect(transferFunction.y, add2.u2) annotation (Line(points={{-99,62},{-88,
                      62},{-88,42},{-32,42}}, color={0,0,127}));
              connect(product.y, top) annotation (Line(points={{47,34},{56,34},{56,58},{73,
                      58}}, color={0,0,127}));
              connect(add.y, bottom) annotation (Line(points={{-3,-74},{30,-74},{30,-40},{
                      69,-40}}, color={0,0,127}));
              annotation (
                Diagram(coordinateSystem(extent={{-140,-100},{220,120}})),
                Icon(coordinateSystem(extent={{-140,-100},{220,120}})));
            end DiscreteMeanValueReal;

            model DiscreteMeanValueImaginary
              Modelica.Blocks.Interfaces.RealInput u
                annotation (Placement(transformation(extent={{-180,-20},{-140,20}})));
              Modelica.Blocks.Interfaces.RealOutput y
                annotation (Placement(transformation(extent={{220,-10},{240,10}})));
              Modelica.Blocks.Discrete.UnitDelay unitDelay(samplePeriod=50e-5)
                annotation (Placement(transformation(extent={{-88,-88},{-68,-68}})));
              Modelica.Blocks.Math.Gain gain(k=2)
                annotation (Placement(transformation(extent={{-58,-88},{-38,-68}})));
              Modelica.Blocks.Math.Gain gain1(k=50)
                annotation (Placement(transformation(extent={{-54,-48},{-34,-28}})));
              Modelica.Blocks.Math.Add add(k2=-1)
                annotation (Placement(transformation(extent={{-24,-84},{-4,-64}})));
              Modelica.Blocks.Math.Add add1
                annotation (Placement(transformation(extent={{86,-8},{106,12}})));
              Modelica.Blocks.Logical.Switch switch1
                annotation (Placement(transformation(extent={{158,-30},{178,-10}})));
              Modelica.Blocks.Sources.Constant const(k=50)
                annotation (Placement(transformation(extent={{-24,4},{-4,24}})));
              Modelica.Blocks.Sources.Constant const1(k=1)
                annotation (Placement(transformation(extent={{98,-40},{118,-20}})));
              Modelica.Blocks.Math.Add add2(k1=-1)
                annotation (Placement(transformation(extent={{-30,38},{-10,58}})));
              Modelica.Blocks.Sources.BooleanStep booleanStep(                  startValue=
                    false, startTime=1/50)
                           annotation (Placement(transformation(extent={{98,26},{118,46}})));
              Modelica.Blocks.Math.Product product
                annotation (Placement(transformation(extent={{26,24},{46,44}})));
              Modelica.Blocks.Discrete.TransferFunction transferFunction(
                samplePeriod=50e-5,
                b={50e-5,50e-5},
                a={2,-2})
                annotation (Placement(transformation(extent={{-120,52},{-100,72}})));
              Modelica.Blocks.Nonlinear.VariableDelay variableDelay(delayMax=2 + 50e-5)
                annotation (Placement(transformation(extent={{-78,58},{-58,78}})));
              Modelica.Blocks.Sources.Constant const2(k=2)
                annotation (Placement(transformation(extent={{-112,24},{-92,44}})));
              Modelica.Blocks.Interfaces.RealOutput top
                annotation (Placement(transformation(extent={{62,48},{84,68}})));
              Modelica.Blocks.Interfaces.RealOutput bottom
                annotation (Placement(transformation(extent={{58,-50},{80,-30}})));
            equation
              connect(u, unitDelay.u) annotation (Line(points={{-160,0},{-94,0},{-94,-78},{
                      -90,-78}}, color={0,0,127}));
              connect(gain1.y, add.u1) annotation (Line(points={{-33,-38},{-30,-38},{-30,
                      -68},{-26,-68}}, color={0,0,127}));
              connect(gain.y, add.u2) annotation (Line(points={{-37,-78},{-32,-78},{-32,-80},
                      {-26,-80}}, color={0,0,127}));
              connect(unitDelay.y, gain.u) annotation (Line(points={{-67,-78},{-60,-78}},
                                           color={0,0,127}));
              connect(gain1.u, unitDelay.u) annotation (Line(points={{-56,-38},{-94,-38},{
                      -94,-78},{-90,-78}}, color={0,0,127}));
              connect(add.y, add1.u2) annotation (Line(points={{-3,-74},{10,-74},{10,-4},{
                      84,-4}}, color={0,0,127}));
              connect(switch1.y, y) annotation (Line(points={{179,-20},{182,-20},{182,0},{
                      230,0}}, color={0,0,127}));
              connect(booleanStep.y, switch1.u2) annotation (Line(points={{119,36},{136,36},
                      {136,-20},{156,-20}}, color={255,0,255}));
              connect(const1.y, switch1.u3) annotation (Line(points={{119,-30},{136,-30},{
                      136,-28},{156,-28}}, color={0,0,127}));
              connect(add1.y, switch1.u1) annotation (Line(points={{107,2},{130,2},{130,-12},
                      {156,-12}}, color={0,0,127}));
              connect(product.y, add1.u1)
                annotation (Line(points={{47,34},{62,34},{62,8},{84,8}}, color={0,0,127}));
              connect(const.y, product.u2)
                annotation (Line(points={{-3,14},{6,14},{6,28},{24,28}}, color={0,0,127}));
              connect(transferFunction.u, unitDelay.u) annotation (Line(points={{-122,62},{
                      -124,62},{-124,0},{-94,0},{-94,-78},{-90,-78}}, color={0,0,127}));
              connect(product.u1, add2.y)
                annotation (Line(points={{24,40},{6,40},{6,48},{-9,48}}, color={0,0,127}));
              connect(transferFunction.y, variableDelay.u) annotation (Line(points={{-99,62},
                      {-90,62},{-90,68},{-80,68}}, color={0,0,127}));
              connect(variableDelay.y, add2.u1) annotation (Line(points={{-57,68},{-48,68},
                      {-48,54},{-32,54}}, color={0,0,127}));
              connect(const2.y, variableDelay.delayTime) annotation (Line(points={{-91,34},
                      {-86,34},{-86,62},{-80,62}}, color={0,0,127}));
              connect(transferFunction.y, add2.u2) annotation (Line(points={{-99,62},{-88,
                      62},{-88,42},{-32,42}}, color={0,0,127}));
              connect(product.y, top) annotation (Line(points={{47,34},{56,34},{56,58},{73,
                      58}}, color={0,0,127}));
              connect(add.y, bottom) annotation (Line(points={{-3,-74},{30,-74},{30,-40},{
                      69,-40}}, color={0,0,127}));
              annotation (
                Diagram(coordinateSystem(extent={{-140,-100},{220,120}})),
                Icon(coordinateSystem(extent={{-140,-100},{220,120}})));
            end DiscreteMeanValueImaginary;

            model ElementWiseMult
              Modelica.Blocks.Interfaces.RealInput u
                annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
              Modelica.Blocks.Interfaces.RealInput u1
                annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
              Modelica.Blocks.Interfaces.RealOutput y
                annotation (Placement(transformation(extent={{100,-10},{120,10}})));
            equation
              y = u1*u;
              annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                    graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={
                          28,108,200})}),                                    Diagram(
                    coordinateSystem(preserveAspectRatio=false), graphics={
                      Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,
                          200})}));
            end ElementWiseMult;

            model Fault
              Modelica.Electrical.Analog.Interfaces.Pin pin
                annotation (Placement(transformation(extent={{-8,-110},{12,-90}})));
              Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch FaultSwitch(Ron=100)
                annotation (Placement(transformation(
                    extent={{-10,-10},{10,10}},
                    rotation=-90,
                    origin={-20,26})));
              Modelica.Electrical.Analog.Basic.Ground ground2
                annotation (Placement(transformation(extent={{-30,-20},{-10,0}})));
              Modelica.Blocks.Sources.BooleanStep booleanStep(startValue=true,
                  startTime=2)
                annotation (Placement(transformation(extent={{28,16},{8,36}})));
            equation
              connect(FaultSwitch.control,booleanStep. y) annotation (Line(points={{-13,26},
                      {7,26}},                            color={255,0,255}));
              connect(ground2.p, FaultSwitch.n)
                annotation (Line(points={{-20,0},{-20,16}}, color={0,0,255}));
              connect(FaultSwitch.p, pin) annotation (Line(points={{-20,36},{-20,48},
                      {-30,48},{-30,-76},{2,-76},{2,-100}}, color={0,0,255}));
              annotation (Icon(coordinateSystem(preserveAspectRatio=false),
                    graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={
                          28,108,200})}),
                  Diagram(coordinateSystem(preserveAspectRatio=false)));
            end Fault;

            model CurrentSensor
              Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation (
                  Placement(transformation(extent={{-108,-10},{-88,10}})));
              Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
                annotation (Placement(transformation(extent={{90,-12},{110,8}})));
              Modelica.Blocks.Interfaces.RealOutput y
                annotation (Placement(transformation(extent={{4,86},{24,106}}),
                    iconTransformation(extent={{4,86},{24,106}})));
            end CurrentSensor;
          end Components;

          model DiscreteFourier
            import RelayModel;
            import RelayModel;
            Modelica.Blocks.Interfaces.RealInput Input
              annotation (Placement(transformation(extent={{-240,70},{-200,110}})));
            Modelica.Blocks.Interfaces.RealOutput Magnitude
              annotation (Placement(transformation(extent={{66,80},{86,100}})));
            Modelica.Blocks.Interfaces.RealOutput Phase
              annotation (Placement(transformation(extent={{86,56},{106,76}})));
            Modelica.Blocks.Math.Gain gain(k=2*Modelica.Constants.pi*1*50)
              annotation (Placement(transformation(extent={{-154,10},{-134,30}})));
            Modelica.Blocks.Math.Sin sin
              annotation (Placement(transformation(extent={{-110,24},{-90,44}})));
            Modelica.Blocks.Math.Cos cos
              annotation (Placement(transformation(extent={{-110,-2},{-90,18}})));
            Modelica.Blocks.Sources.Clock clock
              annotation (Placement(transformation(extent={{-184,10},{-164,30}})));
            Modelica.Blocks.Routing.Multiplex2 multiplex2_1
              annotation (Placement(transformation(extent={{-68,18},{-48,38}})));
            Modelica.Blocks.Math.Gain gain2(k=180/Modelica.Constants.pi)
              annotation (Placement(transformation(extent={{46,56},{66,76}})));
            Modelica.Blocks.Math.RectangularToPolar rectangularToPolar
              annotation (Placement(transformation(extent={{-12,74},{8,94}})));
            Components.DiscreteMeanValue discreteMeanValue(Ts=50e-5)
              annotation (Placement(transformation(extent={{-74,88},{-38,110}})));
            Components.DiscreteMeanValue discreteMeanValue1(Ts=50e-5)
              annotation (Placement(transformation(extent={{-74,62},{-38,84}})));
            Components.ElementWiseMult elementWiseMult
              annotation (Placement(transformation(extent={{-138,74},{-118,94}})));
            Components.ElementWiseMult elementWiseMult1
              annotation (Placement(transformation(extent={{-28,12},{-8,32}})));
            Modelica.Blocks.Sources.Constant const(k=2)
              annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
          equation
            connect(clock.y, gain.u) annotation (Line(points={{-163,20},{-156,20}},
                                      color={0,0,127}));
            connect(rectangularToPolar.y_arg, gain2.u) annotation (Line(points={{9,78},{
                    36,78},{36,66},{44,66}},   color={0,0,127}));
            connect(rectangularToPolar.y_abs, Magnitude) annotation (Line(points={{9,90},{
                    76,90}},                   color={0,0,127}));
            connect(sin.y, multiplex2_1.u1[1]) annotation (Line(points={{-89,34},{
                    -70,34}},                  color={0,0,127}));
            connect(cos.y, multiplex2_1.u2[1]) annotation (Line(points={{-89,8},{
                    -84.5,8},{-84.5,22},{-70,22}},
                                                 color={0,0,127}));
            connect(gain.y, sin.u) annotation (Line(points={{-133,20},{-130,20},{
                    -130,34},{-112,34}},
                                      color={0,0,127}));
            connect(discreteMeanValue1.y, rectangularToPolar.u_im) annotation (Line(
                  points={{-37,72},{-24,72},{-24,78},{-14,78}}, color={0,0,127}));
            connect(elementWiseMult.u, Input)
              annotation (Line(points={{-140,90},{-220,90}}, color={0,0,127}));
            connect(elementWiseMult1.u, multiplex2_1.y[2]) annotation (Line(points={{-30,28},
                    {-47,28}},                                 color={0,0,127}));
            connect(const.y, elementWiseMult1.u1) annotation (Line(points={{-53,0},
                    {-36,0},{-36,16},{-30,16}},     color={0,0,127}));
            connect(gain2.y, Phase)
              annotation (Line(points={{67,66},{96,66}}, color={0,0,127}));
            connect(cos.u, sin.u) annotation (Line(points={{-112,8},{-130,8},{-130,
                    34},{-112,34}}, color={0,0,127}));
            connect(elementWiseMult1.y, elementWiseMult.u1) annotation (Line(points=
                   {{-7,22},{0,22},{0,56},{-152,56},{-152,78},{-140,78}}, color={0,
                    0,127}));
            connect(elementWiseMult.y, discreteMeanValue.u) annotation (Line(points=
                   {{-117,84},{-86,84},{-86,98},{-76,98}}, color={0,0,127}));
            connect(discreteMeanValue1.u, discreteMeanValue.u) annotation (Line(
                  points={{-76,72},{-86,72},{-86,98},{-76,98}}, color={0,0,127}));
            connect(discreteMeanValue.y, rectangularToPolar.u_re) annotation (Line(
                  points={{-37,98},{-24,98},{-24,90},{-14,90}}, color={0,0,127}));
            annotation (
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{100,
                      120}})),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
                      100,120}})));
          end DiscreteFourier;
        end DiscreteFourier;

        model toThePower
          Modelica.Blocks.Interfaces.RealInput u
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Interfaces.RealInput alpha
            annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
        equation
          y= u^alpha;
          annotation (
            Icon(coordinateSystem(preserveAspectRatio=false)),
            Diagram(coordinateSystem(preserveAspectRatio=false)));
        end toThePower;

        block ElementDivision "Output first input divided by second input"
        //extends Interfaces.SI2SO;
          Modelica.Blocks.Interfaces.RealInput u
            annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
          Modelica.Blocks.Interfaces.RealInput u1
            annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        equation
        y = u./u1;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
        end ElementDivision;

        model Y3
          Modelica.Blocks.Interfaces.RealInput x
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
          Modelica.Blocks.Interfaces.RealInput eps
            annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
        equation
         //y=smooth(1, if x>eps then 1/x else 1/eps);
         //y= noEvent(if x>eps then 1/x else 1/eps);
          //y=1./max(x,eps);
          y=1./eps;
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end Y3;

        model EquationCalculatingOperatingTime
          Modelica.Blocks.Interfaces.RealInput Control
            annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
          Modelica.Blocks.Interfaces.RealInput CurrentInput
            annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
          Modelica.Blocks.Interfaces.RealOutput OperatingTime
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          parameter Real alpha=0.02 "Constant output value";
          parameter Real TMS=0.5 "Constant output value";
          parameter Real C=0.14 "Constant output value";
          parameter Real ls=1 "Constant output value";
          Real Den;
        equation
          Den = 1./max(((CurrentInput/ls)^alpha)-1,0.0247);
          OperatingTime = (C *Den)*TMS;
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end EquationCalculatingOperatingTime;

        model TwoDiscreteFourier
          import RelayPack;
          Modelica.Blocks.Math.Gain gain(k=2*Modelica.Constants.pi*1*50)
            annotation (Placement(transformation(extent={{-284,-12},{-264,8}})));
          Modelica.Blocks.Math.Sin sin
            annotation (Placement(transformation(extent={{-244,6},{-224,26}})));
          Modelica.Blocks.Math.Cos cos
            annotation (Placement(transformation(extent={{-244,-20},{-224,0}})));
          Modelica.Blocks.Sources.Clock clock
            annotation (Placement(transformation(extent={{-316,-12},{-296,8}})));
          Modelica.Blocks.Math.Gain gain2(k=180/Modelica.Constants.pi)
            annotation (Placement(transformation(extent={{28,30},{48,50}})));
          Modelica.Blocks.Math.RectangularToPolar rectangularToPolar
            annotation (Placement(transformation(extent={{-66,36},{-46,56}})));
          RelayPack.Components.DiscreteFourier.Components.DiscreteMeanValue discreteMeanValue(
            FreqHz=FreqHz,
            Ts=Ts,
            Vinit=Vinit,
            booleanStep(startTime=1/FreqHz),
            gain1(k=0),
            gain(k=0),
            Delay=0.02) annotation (Placement(transformation(extent={{-166,64},
                    {-130,86}})));
          RelayPack.Components.DiscreteFourier.Components.ElementWiseMult intoreal
            annotation (Placement(transformation(extent={{-250,64},{-230,84}})));
          Modelica.Blocks.Interfaces.RealInput Input
            annotation (Placement(transformation(extent={{-358,60},{-318,100}})));
          Modelica.Blocks.Interfaces.RealOutput Magnitude
            annotation (Placement(transformation(extent={{60,66},{88,94}})));
          Modelica.Blocks.Interfaces.RealOutput Phase
            annotation (Placement(transformation(extent={{60,24},{92,56}})));
          Modelica.Blocks.Math.Gain gain1(k=2) annotation (Placement(transformation(
                  extent={{-198,10},{-178,30}})));
          Modelica.Blocks.Math.Gain gain3(k=2) annotation (Placement(transformation(
                  extent={{-192,-26},{-172,-6}})));
          RelayPack.Components.DiscreteFourier.Components.ElementWiseMult intoimag
            annotation (Placement(transformation(extent={{-250,38},{-230,58}})));
          parameter Real FreqHz=50 "Constant output value";
          parameter Modelica.SIunits.Time Ts=5e-5 "Sample period of component";
          parameter Real Vinit=1 "Constant output value";
          RelayPack.Components.DiscreteFourier.Components.DiscreteMeanValue discreteMeanValue2(
            FreqHz=FreqHz,
            Ts=Ts,
            Vinit=Vinit,
            booleanStep(startTime=1/FreqHz),
            gain1(k=0),
            gain(k=0),
            Delay=0.02) annotation (Placement(transformation(extent={{-166,38},
                    {-130,60}})));
        equation
          connect(clock.y,gain. u) annotation (Line(points={{-295,-2},{-286,-2}},
                                    color={0,0,127}));
          connect(rectangularToPolar.y_arg,gain2. u) annotation (Line(points={{-45,40},
                  {26,40}},                  color={0,0,127}));
          connect(gain.y,sin. u) annotation (Line(points={{-263,-2},{-258,-2},{-258,
                  16},{-246,16}},   color={0,0,127}));
          connect(cos.u,sin. u) annotation (Line(points={{-246,-10},{-258,-10},{
                  -258,16},{-246,16}},
                                    color={0,0,127}));
          connect(discreteMeanValue.y,rectangularToPolar. u_re) annotation (Line(
                points={{-129,74},{-100,74},{-100,52},{-68,52}},
                                                              color={0,0,127}));
          connect(intoreal.u, Input)
            annotation (Line(points={{-252,80},{-338,80}}, color={0,0,127}));
          connect(sin.y, gain1.u) annotation (Line(points={{-223,16},{-212,16},{
                  -212,20},{-200,20}},      color={0,0,127}));
          connect(cos.y, gain3.u) annotation (Line(points={{-223,-10},{-208,-10},{
                  -208,-16},{-194,-16}},    color={0,0,127}));
          connect(intoimag.u, Input) annotation (Line(points={{-252,54},{-288,54},{
                  -288,80},{-338,80}}, color={0,0,127}));
          connect(intoreal.y, discreteMeanValue.u) annotation (Line(points={{-229,74},
                  {-168,74}},                         color={0,0,127}));
          connect(intoimag.y, discreteMeanValue2.u) annotation (Line(points={{-229,48},
                  {-168,48}},                         color={0,0,127}));
          connect(discreteMeanValue2.y, rectangularToPolar.u_im) annotation (Line(
                points={{-129,48},{-76,48},{-76,40},{-68,40}}, color={0,0,127}));
          connect(gain2.y, Phase)
            annotation (Line(points={{49,40},{76,40}}, color={0,0,127}));
          connect(gain1.y, intoimag.u1) annotation (Line(points={{-177,20},{-170,20},
                  {-170,34},{-260,34},{-260,42},{-252,42}}, color={0,0,127}));
          connect(gain3.y, intoreal.u1) annotation (Line(points={{-171,-16},{-156,
                  -16},{-156,32},{-266,32},{-266,68},{-252,68}}, color={0,0,127}));
          connect(rectangularToPolar.y_abs, Magnitude) annotation (Line(points={{
                  -45,52},{14,52},{14,80},{74,80}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-700,
                    -380},{480,440}}), graphics={Rectangle(extent={{-320,100},{100,
                      -218}}, lineColor={28,108,200})}),
                                             Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-700,-380},{480,440}}),
                graphics={
                Text(
                  extent={{-346,10},{-326,-2}},
                  lineColor={0,0,0},
                  textString="(a)"),
                Text(
                  extent={{-250,80},{-230,68}},
                  lineColor={0,0,0},
                  textString="(b)"),
                Text(
                  extent={{-250,54},{-230,42}},
                  lineColor={0,0,0},
                  textString="(bi)"),
                Text(
                  extent={{-158,82},{-138,70}},
                  lineColor={0,0,0},
                  textString="(c)"),
                Text(
                  extent={{-66,74},{-46,62}},
                  lineColor={0,0,0},
                  textString="(d)"),
                Text(
                  extent={{-156,56},{-136,44}},
                  lineColor={0,0,0},
                  textString="(ci)"),
                Line(points={{-316,32},{-326,32},{-326,-30},{-316,-30}}, color={28,
                      108,200})}));
        end TwoDiscreteFourier;

      end Components;

      model ReferenceRelay
        Modelica.Blocks.Math.Add add(       k2=1, k1=-1)
          annotation (Placement(transformation(extent={{-130,30},{-110,50}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-252,-10},{-232,10}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-214,-10},{-194,10}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-286,-52})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-296,-10},{-276,10}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-334,-62})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-172,34},{-152,54}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-174,-14},{-154,6}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-180,110}),
                                iconTransformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-10,0})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Components.CalculatingOperationTime calculatingOperationTime(
          ls=0.73,
          k=0.041,
          TMS=0.21492)
          annotation (Placement(transformation(extent={{-174,-56},{-154,-36}})));
        parameter Real k= 0.041 "Constant output value";

        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop
          annotation (Placement(transformation(extent={{-54,-16},{-34,4}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
          annotation (Placement(transformation(extent={{-98,-46},{-78,-26}})));
        Modelica.Blocks.Math.Division division
          annotation (Placement(transformation(extent={{-332,-8},{-312,12}})));
        Modelica.Blocks.Sources.Constant const2(k=0.22)        annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-342,-32})));
        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop1
          annotation (Placement(transformation(extent={{-232,-42},{-212,-22}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant1(k=false)
          annotation (Placement(transformation(extent={{-260,-54},{-240,-34}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-420,-20},{-380,20}})));
      equation

        connect(const1.y, product.u2) annotation (Line(points={{-323,-62},{-308,
                -62},{-308,-6},{-298,-6}}, color={0,0,127}));
        connect(product.y, greater1.u1)
          annotation (Line(points={{-275,0},{-254,0}}, color={0,0,127}));
        connect(const.y, greater1.u2) annotation (Line(points={{-275,-52},{-262,
                -52},{-262,-8},{-254,-8}}, color={0,0,127}));
        connect(booleanToReal1.y, timer.u) annotation (Line(points={{-193,0},{
                -184,0},{-184,0.2},{-176,0.2}}, color={0,0,127}));
        connect(extractingTimeOfFault.u, timer.u) annotation (Line(points={{-174,44},
                {-184,44},{-184,0.2},{-176,0.2}},          color={0,0,127}));
        connect(calculatingOperationTime.Control, timer.u) annotation (Line(
              points={{-176,-40},{-184,-40},{-184,0.2},{-176,0.2}}, color={0,0,
                127}));
        connect(extractingTimeOfFault.y, add.u1)
          annotation (Line(points={{-151,44},{-140,44},{-140,46},{-132,46}},
                                                         color={0,0,127}));
        connect(timer.y, add.u2) annotation (Line(points={{-153,-4},{-138,-4},{
                -138,34},{-132,34}}, color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{-109,40},{-100,40},
                {-100,0},{-94,0}}, color={0,0,127}));
        connect(calculatingOperationTime.OperationTime, greater2.u2)
          annotation (Line(points={{-153,-46},{-124,-46},{-124,-8},{-94,-8}},
              color={0,0,127}));
        connect(greater2.y, rSFlipFlop.S)
          annotation (Line(points={{-71,0},{-56,0}}, color={255,0,255}));
        connect(booleanConstant.y, rSFlipFlop.R) annotation (Line(points={{-77,-36},
                {-62,-36},{-62,-12},{-56,-12}},      color={255,0,255}));
        connect(rSFlipFlop.Q, TripSingal) annotation (Line(points={{-33,0},{-28,
                0},{-28,80},{-180,80},{-180,110}}, color={255,0,255}));
        connect(calculatingOperationTime.CurrentInput, greater1.u1) annotation (
           Line(points={{-176,-52},{-238,-52},{-238,-78},{-300,-78},{-300,-24},
                {-266,-24},{-266,0},{-254,0}}, color={0,0,127}));
        connect(division.u2, const2.y) annotation (Line(points={{-334,-4},{-350,
                -4},{-350,-32},{-331,-32}}, color={0,0,127}));
        connect(division.y, product.u1) annotation (Line(points={{-311,2},{-304,
                2},{-304,6},{-298,6}}, color={0,0,127}));
        connect(rSFlipFlop1.Q, booleanToReal1.u) annotation (Line(points={{-211,
                -26},{-202,-26},{-202,-16},{-216,-16},{-216,0}}, color={255,0,
                255}));
        connect(rSFlipFlop1.S, greater1.y) annotation (Line(points={{-234,-26},
                {-240,-26},{-240,-14},{-226,-14},{-226,0},{-231,0}}, color={255,
                0,255}));
        connect(booleanConstant1.y, rSFlipFlop1.R) annotation (Line(points={{
                -239,-44},{-236,-44},{-236,-38},{-234,-38}}, color={255,0,255}));
        connect(division.u1, u) annotation (Line(points={{-334,8},{-360,8},{-360,0},{-400,
                0}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},
                  {-20,100}}),       graphics={Rectangle(extent={{-380,100},{
                    -20,-100}},
                            lineColor={28,108,200}), Text(
                extent={{-290,56},{-96,-46}},
                lineColor={28,108,200},
                textString="Relay")}),                                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},{-20,100}}),
                          graphics={
              Text(
                extent={{-252,26},{-232,14}},
                lineColor={0,0,0},
                textString="(e)"),
              Text(
                extent={{-172,52},{-152,40}},
                lineColor={0,0,0},
                textString="(f)"),
              Text(
                extent={{-174,2},{-154,-10}},
                lineColor={0,0,0},
                textString="(g)"),
              Text(
                extent={{-174,-40},{-154,-52}},
                lineColor={0,0,0},
                textString="(h)")}));
      end ReferenceRelay;

      package Data
        package Records
          record AlphaData
           parameter Real alpha;
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end AlphaData;

          record CData
             parameter Real C;
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end CData;

          record RelayData "empty complete relay data record"
            extends Modelica.Icons.Record;
            replaceable record Alpha =
            AlphaData constrainedby AlphaData
            annotation (choicesAllMatching);
            Alpha alpha;
            replaceable record C =
            CData constrainedby CData
            annotation (choicesAllMatching);
            C c;
             replaceable record EPS = EPSData constrainedby EPSData
                                             annotation (choicesAllMatching);
            EPS eps;
          end RelayData;

          record EPSData
           parameter Real eps;
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end EPSData;
        end Records;

        package RelayData
          record StandardInverseData = Records.RelayData (
              redeclare replaceable record Alpha = AlphaData.SIAlpha,
              redeclare replaceable record C = CData.SIC,
              redeclare replaceable record EPS = EPSData.SIEPS);
          record VeryInverseData = Records.RelayData (
              redeclare replaceable record Alpha = AlphaData.VIAlpha,
              redeclare replaceable record C = CData.VIC,
              redeclare replaceable record EPS = EPSData.VIEPS);
          record ExtremelyInverseData = Records.RelayData (
              redeclare replaceable record Alpha = AlphaData.EIAlpha,
              redeclare replaceable record C = CData.EIC,
              redeclare replaceable record EPS = EPSData.EIEPS);
          record LongInverseData = Records.RelayData (
              redeclare replaceable record Alpha = AlphaData.LIAlpha,
              redeclare replaceable record C = CData.LIC,
              redeclare replaceable record EPS = EPSData.LEPS);
        end RelayData;

        package AlphaData
          record SIAlpha
            extends Records.AlphaData(alpha=0.02);
          end SIAlpha;

          record VIAlpha
            extends Records.AlphaData(alpha=1);
          end VIAlpha;

          record EIAlpha
            extends Records.AlphaData(alpha=2);
          end EIAlpha;

          record LIAlpha
            extends Records.AlphaData(alpha=1);
          end LIAlpha;
        end AlphaData;

        package CData
          record SIC
            extends Records.CData(C=0.14);
          end SIC;

          record VIC
            extends Records.CData(C=13.5);
          end VIC;

          record EIC
            extends Records.CData(C=80);
          end EIC;

          record LIC
            extends Records.CData(C=120);
          end LIC;
        end CData;

        package EPSData
           record SIEPS
            extends Records.EPSData(eps=0.41);
           end SIEPS;

          record VIEPS
            extends Records.EPSData(eps=0.07);
          end VIEPS;

          record EIEPS
            extends Records.EPSData(eps=0.102);
          end EIEPS;

          record LEPS
            extends Records.EPSData(eps=0);
          end LEPS;
        end EPSData;
      end Data;

      model RecordReferenceRelay
        Modelica.Blocks.Math.Add add(       k2=1, k1=-1)
          annotation (Placement(transformation(extent={{-130,30},{-110,50}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-252,-10},{-232,10}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-214,-10},{-194,10}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-286,-52})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-296,-10},{-276,10}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-334,-62})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-172,34},{-152,54}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-174,-14},{-154,6}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-180,110}),
                                iconTransformation(extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-10,0})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Components.CalculatingOperationTime calculatingOperationTime(
          alpha=alpha,
          TMS=TMS,
          C=C,
          ls=ls,
          k=k)
          annotation (Placement(transformation(extent={{-174,-56},{-154,-36}})));
        parameter Real k= 0.041 "Constant output value";

        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop
          annotation (Placement(transformation(extent={{-54,-16},{-34,4}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
          annotation (Placement(transformation(extent={{-98,-46},{-78,-26}})));
        Modelica.Blocks.Math.Division division
          annotation (Placement(transformation(extent={{-332,-8},{-312,12}})));
        Modelica.Blocks.Sources.Constant const2(k=0.22)        annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-342,-32})));
        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop1
          annotation (Placement(transformation(extent={{-232,-42},{-212,-22}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant1(k=false)
          annotation (Placement(transformation(extent={{-260,-54},{-240,-34}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-420,-20},{-380,20}})));
        parameter Real ls=0.73 "Constant output value";
      equation

        connect(const1.y, product.u2) annotation (Line(points={{-323,-62},{-308,
                -62},{-308,-6},{-298,-6}}, color={0,0,127}));
        connect(product.y, greater1.u1)
          annotation (Line(points={{-275,0},{-254,0}}, color={0,0,127}));
        connect(const.y, greater1.u2) annotation (Line(points={{-275,-52},{-262,
                -52},{-262,-8},{-254,-8}}, color={0,0,127}));
        connect(booleanToReal1.y, timer.u) annotation (Line(points={{-193,0},{
                -184,0},{-184,0.2},{-176,0.2}}, color={0,0,127}));
        connect(extractingTimeOfFault.u, timer.u) annotation (Line(points={{-174,44},
                {-184,44},{-184,0.2},{-176,0.2}},          color={0,0,127}));
        connect(calculatingOperationTime.Control, timer.u) annotation (Line(
              points={{-176,-40},{-184,-40},{-184,0.2},{-176,0.2}}, color={0,0,
                127}));
        connect(extractingTimeOfFault.y, add.u1)
          annotation (Line(points={{-151,44},{-140,44},{-140,46},{-132,46}},
                                                         color={0,0,127}));
        connect(timer.y, add.u2) annotation (Line(points={{-153,-4},{-138,-4},{
                -138,34},{-132,34}}, color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{-109,40},{-100,40},
                {-100,0},{-94,0}}, color={0,0,127}));
        connect(calculatingOperationTime.OperationTime, greater2.u2)
          annotation (Line(points={{-153,-46},{-124,-46},{-124,-8},{-94,-8}},
              color={0,0,127}));
        connect(greater2.y, rSFlipFlop.S)
          annotation (Line(points={{-71,0},{-56,0}}, color={255,0,255}));
        connect(booleanConstant.y, rSFlipFlop.R) annotation (Line(points={{-77,-36},
                {-62,-36},{-62,-12},{-56,-12}},      color={255,0,255}));
        connect(rSFlipFlop.Q, TripSingal) annotation (Line(points={{-33,0},{-28,
                0},{-28,80},{-180,80},{-180,110}}, color={255,0,255}));
        connect(calculatingOperationTime.CurrentInput, greater1.u1) annotation (
           Line(points={{-176,-52},{-238,-52},{-238,-78},{-300,-78},{-300,-24},
                {-266,-24},{-266,0},{-254,0}}, color={0,0,127}));
        connect(division.u2, const2.y) annotation (Line(points={{-334,-4},{-350,
                -4},{-350,-32},{-331,-32}}, color={0,0,127}));
        connect(division.y, product.u1) annotation (Line(points={{-311,2},{-304,
                2},{-304,6},{-298,6}}, color={0,0,127}));
        connect(rSFlipFlop1.Q, booleanToReal1.u) annotation (Line(points={{-211,
                -26},{-202,-26},{-202,-16},{-216,-16},{-216,0}}, color={255,0,
                255}));
        connect(rSFlipFlop1.S, greater1.y) annotation (Line(points={{-234,-26},
                {-240,-26},{-240,-14},{-226,-14},{-226,0},{-231,0}}, color={255,
                0,255}));
        connect(booleanConstant1.y, rSFlipFlop1.R) annotation (Line(points={{
                -239,-44},{-236,-44},{-236,-38},{-234,-38}}, color={255,0,255}));
        connect(division.u1, u) annotation (Line(points={{-334,8},{-360,8},{-360,0},{-400,
                0}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},
                  {-20,100}}),       graphics={Rectangle(extent={{-380,100},{
                    -20,-100}},
                            lineColor={28,108,200}), Text(
                extent={{-290,56},{-96,-46}},
                lineColor={28,108,200},
                textString="Relay")}),                                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},{-20,100}}),
                          graphics={
              Text(
                extent={{-252,26},{-232,14}},
                lineColor={0,0,0},
                textString="(e)"),
              Text(
                extent={{-172,52},{-152,40}},
                lineColor={0,0,0},
                textString="(f)"),
              Text(
                extent={{-174,2},{-154,-10}},
                lineColor={0,0,0},
                textString="(g)"),
              Text(
                extent={{-174,-40},{-154,-52}},
                lineColor={0,0,0},
                textString="(h)")}));
      end RecordReferenceRelay;
      annotation ();
    end RelayPack;
  end Components;

  package TestSystems

    model AIOSNoMotorRelay
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=1200,
        Q_0=53)
        annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
      OpenIPSL.Electrical.Buses.Bus OneBus(
        angle_0=0,
        V_b=380,
        V_0=1.08,
        P_0=450,
        Q_0=118)
        annotation (Placement(transformation(extent={{-244,-26},{-224,-6}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0,
        X=0.4143333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-176,-62},{-156,-42}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-156,-26},{-136,-6}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        displayPF=true,
        V_0=pF1_1.voltage.InfiniteBusV_0,
        angle_0=pF1_1.voltage.InfiniteBusangle_0,
        P_0=pF1_1.power.InfiniteBusP_0,
        Q_0=pF1_1.power.InfiniteBusQ_0)
        annotation (Placement(transformation(extent={{-298,-26},{-278,-6}})));
      OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
            transformation(extent={{-174,-102},{-154,-82}})));
      OpenIPSL.Electrical.Buses.Bus FiveBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-15.2,
        P_0=1200,
        Q_0=0)
        annotation (Placement(transformation(extent={{18,-102},{38,-82}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.029999989612,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-28,-102},{-8,-82}})));
      Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
            Data.VoltageData.VPF2, redeclare record Power = Data.PowerData.PPF2)
        annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
      Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
            Data.VoltageData.VPF1, redeclare record Power = Data.PowerData.PPF1)
        annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
      Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
            Data.VoltageData.VPF3, redeclare record Power = Data.PowerData.PPF3)
        annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
      Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
            Data.VoltageData.VPF5, redeclare record Power = Data.PowerData.PPF5)
        annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
      Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
            Data.VoltageData.VPF6, redeclare record Power = Data.PowerData.PPF6)
        annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
      Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
            Data.VoltageData.VPF7, redeclare record Power = Data.PowerData.PPF7)
        annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
      OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
        annotation (Placement(transformation(extent={{-202,-26},{-182,-6}})));
      OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
        annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
      .OCRAIOSPSSE.Components.PSSEGeneratorTGOV generatorTGOV(
        V_b=20,
        M_b=750,
        V_0=pF1_1.voltage.GeneratorV_0,
        angle_0=pF1_1.voltage.Generatorangle_0,
        P_0=pF1_1.power.GeneratorP_0,
        Q_0=pF1_1.power.GeneratorQ_0)
        annotation (Placement(transformation(extent={{-214,-106},{-196,-82}})));
      OpenIPSL.Electrical.Events.PwFault pwFault(
        R=0,
        X=0.0375,
        t1=1,
        t2=1.13999)
               annotation (Placement(transformation(extent={{-116,-46},{-104,-34}})));

     Real Imag;
      Modelica.Blocks.Sources.RealExpression realExpression(y=Imag)
        annotation (Placement(transformation(extent={{-202,12},{-182,32}})));
      Components.RelayPack.ReferenceRelay ReferenceRelay
        annotation (Placement(transformation(extent={{-166,16},{-140,28}})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer
        twoWindingTransformer(
        G=0,
        B=0,
        VB1=20,
        VB2=380,
        CW=1,
        CZ=1,
        X=0.12,
        R=0,
        t2=1.04)
        annotation (Placement(transformation(extent={{-118,-96},{-106,-88}})));
      OpenIPSL.Electrical.Loads.PSSE.Load load(
        V_b=380,
        V_0=pF1_1.voltage.PQLoadV_0,
        angle_0=pF1_1.voltage.PQLoadangle_0,
        P_0=pF1_1.power.PQLoadP_0,
        Q_0=pF1_1.power.PQLoadQ_0)
        annotation (Placement(transformation(extent={{72,-130},{100,-104}})));
    equation
      Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
      connect(FiveBus.p, pwLine2.n)
        annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
      connect(pwLine2.p, ThreeBus.p)
        annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
      connect(pwLine3.p, breaker.r)
        annotation (Line(points={{-155,-16},{-182,-16}}, color={0,0,255}));
      connect(breaker1.Trigger, breaker.Trigger) annotation (Line(points={{-106,-4},
              {-106,4},{-192,4},{-192,-4}},          color={255,0,255}));
      connect(ThreeBus.p, breaker1.r) annotation (Line(points={{-72,-92},{-72,
              -16},{-96,-16}},                    color={0,0,255}));
      connect(generatorTGOV.pwPin, TwoBus.p) annotation (Line(points={{-195.64,
              -92.2},{-178.44,-92.2},{-178.44,-92},{-164,-92}},     color={0,
              0,255}));
      connect(infiniteBus.p, OneBus.p)
        annotation (Line(points={{-278,-16},{-234,-16}},
                                                       color={0,0,255}));
      connect(pwLine1.n, breaker1.r) annotation (Line(points={{-157,-52},{-86,
              -52},{-86,-16},{-96,-16}}, color={0,0,255}));
      connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
              -16},{-226,-52},{-175,-52}},
                                        color={0,0,255}));
      connect(pwFault.p, pwLine3.n) annotation (Line(points={{-117,-40},{-126,-40},{
              -126,-16},{-137,-16}}, color={0,0,255}));
      connect(breaker1.s, pwLine3.n)
        annotation (Line(points={{-116,-16},{-137,-16}}, color={0,0,255}));
      connect(breaker.s, pwLine1.p) annotation (Line(points={{-202,-16},{-226,
              -16},{-226,-52},{-175,-52}}, color={0,0,255}));
      connect(realExpression.y, ReferenceRelay.u)
        annotation (Line(points={{-181,22},{-167.444,22}}, color={0,0,127}));
      connect(ReferenceRelay.TripSingal, breaker.Trigger) annotation (Line(
            points={{-139.278,22},{-134,22},{-134,4},{-192,4},{-192,-4}},
            color={255,0,255}));
      connect(TwoBus.p, twoWindingTransformer.p)
        annotation (Line(points={{-164,-92},{-119,-92}}, color={0,0,255}));
      connect(ThreeBus.p, twoWindingTransformer.n)
        annotation (Line(points={{-72,-92},{-105,-92}}, color={0,0,255}));
      connect(load.p, FiveBus.p) annotation (Line(points={{86,-104},{86,-92},
              {28,-92}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                -140},{140,80}})),      Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
    end AIOSNoMotorRelay;

    model AIOSNoMotor
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=1200,
        Q_0=53)
        annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
      OpenIPSL.Electrical.Buses.Bus OneBus(
        angle_0=0,
        V_b=380,
        V_0=1.08,
        P_0=450,
        Q_0=118)
        annotation (Placement(transformation(extent={{-244,-26},{-224,-6}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0,
        X=0.4143333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-176,-62},{-156,-42}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-176,-26},{-156,-6}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        displayPF=true,
        V_0=pF1_1.voltage.InfiniteBusV_0,
        angle_0=pF1_1.voltage.InfiniteBusangle_0,
        P_0=pF1_1.power.InfiniteBusP_0,
        Q_0=pF1_1.power.InfiniteBusQ_0)
        annotation (Placement(transformation(extent={{-298,-26},{-278,-6}})));
      OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
            transformation(extent={{-174,-102},{-154,-82}})));
      OpenIPSL.Electrical.Buses.Bus FiveBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-15.2,
        P_0=1200,
        Q_0=0)
        annotation (Placement(transformation(extent={{18,-102},{38,-82}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.029999989612,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-28,-102},{-8,-82}})));
      Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
            Data.VoltageData.VPF2, redeclare record Power = Data.PowerData.PPF2)
        annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
      Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
            Data.VoltageData.VPF1, redeclare record Power = Data.PowerData.PPF1)
        annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
      Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
            Data.VoltageData.VPF3, redeclare record Power = Data.PowerData.PPF3)
        annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
      Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
            Data.VoltageData.VPF5, redeclare record Power = Data.PowerData.PPF5)
        annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
      Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
            Data.VoltageData.VPF6, redeclare record Power = Data.PowerData.PPF6)
        annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
      Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
            Data.VoltageData.VPF7, redeclare record Power = Data.PowerData.PPF7)
        annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
      .OCRAIOSPSSE.Components.PSSEGeneratorTGOV generatorTGOV(
        V_b=20,
        M_b=750,
        V_0=pF1_1.voltage.GeneratorV_0,
        angle_0=pF1_1.voltage.Generatorangle_0,
        P_0=pF1_1.power.GeneratorP_0,
        Q_0=pF1_1.power.GeneratorQ_0)
        annotation (Placement(transformation(extent={{-214,-106},{-196,-82}})));

     Real Imag;
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer
        twoWindingTransformer(
        G=0,
        B=0,
        VB1=20,
        VB2=380,
        CW=1,
        CZ=1,
        X=0.12,
        R=0,
        t2=1.04)
        annotation (Placement(transformation(extent={{-118,-96},{-106,-88}})));
      OpenIPSL.Electrical.Loads.PSSE.Load load(
        V_b=380,
        V_0=pF1_1.voltage.PQLoadV_0,
        angle_0=pF1_1.voltage.PQLoadangle_0,
        P_0=pF1_1.power.PQLoadP_0,
        Q_0=pF1_1.power.PQLoadQ_0)
        annotation (Placement(transformation(extent={{72,-130},{100,-104}})));
    equation
      Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
      connect(FiveBus.p, pwLine2.n)
        annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
      connect(pwLine2.p, ThreeBus.p)
        annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
      connect(generatorTGOV.pwPin, TwoBus.p) annotation (Line(points={{-195.64,
              -92.2},{-178.44,-92.2},{-178.44,-92},{-164,-92}},     color={0,
              0,255}));
      connect(infiniteBus.p, OneBus.p)
        annotation (Line(points={{-278,-16},{-234,-16}},
                                                       color={0,0,255}));
      connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
              -16},{-226,-52},{-175,-52}},
                                        color={0,0,255}));
      connect(TwoBus.p, twoWindingTransformer.p)
        annotation (Line(points={{-164,-92},{-119,-92}}, color={0,0,255}));
      connect(ThreeBus.p, twoWindingTransformer.n)
        annotation (Line(points={{-72,-92},{-105,-92}}, color={0,0,255}));
      connect(load.p, FiveBus.p) annotation (Line(points={{86,-104},{86,-92},
              {28,-92}}, color={0,0,255}));
      connect(pwLine3.p, pwLine1.p) annotation (Line(points={{-175,-16},{-226,
              -16},{-226,-52},{-175,-52}}, color={0,0,255}));
      connect(pwLine1.n, pwLine3.n) annotation (Line(points={{-157,-52},{-118,
              -52},{-118,-16},{-157,-16}}, color={0,0,255}));
      connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-72,-92},{-72,
              -32},{-118,-32},{-118,-16},{-157,-16}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                -140},{140,80}})),      Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
    end AIOSNoMotor;

    model AIOSRecordNoMotorRelay
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=1200,
        Q_0=53)
        annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
      OpenIPSL.Electrical.Buses.Bus OneBus(
        angle_0=0,
        V_b=380,
        V_0=1.08,
        P_0=450,
        Q_0=118)
        annotation (Placement(transformation(extent={{-244,-26},{-224,-6}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0,
        X=0.4143333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-176,-62},{-156,-42}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-156,-26},{-136,-6}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        displayPF=true,
        V_0=pF1_1.voltage.InfiniteBusV_0,
        angle_0=pF1_1.voltage.InfiniteBusangle_0,
        P_0=pF1_1.power.InfiniteBusP_0,
        Q_0=pF1_1.power.InfiniteBusQ_0)
        annotation (Placement(transformation(extent={{-298,-26},{-278,-6}})));
      OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
            transformation(extent={{-174,-102},{-154,-82}})));
      OpenIPSL.Electrical.Buses.Bus FiveBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-15.2,
        P_0=1200,
        Q_0=0)
        annotation (Placement(transformation(extent={{18,-102},{38,-82}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.029999989612,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-28,-102},{-8,-82}})));
      Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
            Data.VoltageData.VPF2, redeclare record Power = Data.PowerData.PPF2)
        annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
      Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
            Data.VoltageData.VPF1, redeclare record Power = Data.PowerData.PPF1)
        annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
      Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
            Data.VoltageData.VPF3, redeclare record Power = Data.PowerData.PPF3)
        annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
      Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
            Data.VoltageData.VPF5, redeclare record Power = Data.PowerData.PPF5)
        annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
      Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
            Data.VoltageData.VPF6, redeclare record Power = Data.PowerData.PPF6)
        annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
      Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
            Data.VoltageData.VPF7, redeclare record Power = Data.PowerData.PPF7)
        annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
      OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
        annotation (Placement(transformation(extent={{-202,-26},{-182,-6}})));
      OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
        annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
      .OCRAIOSPSSE.Components.PSSEGeneratorTGOV generatorTGOV(
        V_b=20,
        M_b=750,
        V_0=pF1_1.voltage.GeneratorV_0,
        angle_0=pF1_1.voltage.Generatorangle_0,
        P_0=pF1_1.power.GeneratorP_0,
        Q_0=pF1_1.power.GeneratorQ_0)
        annotation (Placement(transformation(extent={{-214,-106},{-196,-82}})));
      OpenIPSL.Electrical.Events.PwFault pwFault(
        R=0,
        X=0.0375,
        t1=1,
        t2=1.13999)
               annotation (Placement(transformation(extent={{-116,-46},{-104,-34}})));

     Real Imag;
      Modelica.Blocks.Sources.RealExpression realExpression(y=Imag)
        annotation (Placement(transformation(extent={{-202,12},{-182,32}})));
      OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer
        twoWindingTransformer(
        G=0,
        B=0,
        VB1=20,
        VB2=380,
        CW=1,
        CZ=1,
        X=0.12,
        R=0,
        t2=1.04)
        annotation (Placement(transformation(extent={{-118,-96},{-106,-88}})));
      OpenIPSL.Electrical.Loads.PSSE.Load load(
        V_b=380,
        V_0=pF1_1.voltage.PQLoadV_0,
        angle_0=pF1_1.voltage.PQLoadangle_0,
        P_0=pF1_1.power.PQLoadP_0,
        Q_0=pF1_1.power.PQLoadQ_0)
        annotation (Placement(transformation(extent={{72,-130},{100,-104}})));
      Components.RelayPack.RecordReferenceRelay recordReferenceRelay(alpha=
            standardInverseData.alpha.alpha, C=standardInverseData.c.C)
        annotation (Placement(transformation(extent={{-154,12},{-130,34}})));
      Components.RelayPack.Data.RelayData.StandardInverseData
        standardInverseData
        annotation (Placement(transformation(extent={{-26,60},{-6,80}})));
      Components.RelayPack.Data.RelayData.VeryInverseData veryInverseData
        annotation (Placement(transformation(extent={{-26,34},{-6,54}})));
      Components.RelayPack.Data.RelayData.ExtremelyInverseData
        extremelyInverseData
        annotation (Placement(transformation(extent={{-26,10},{-6,30}})));
      Components.RelayPack.Data.RelayData.LongInverseData longInverseData
        annotation (Placement(transformation(extent={{-26,-14},{-6,6}})));
    equation
      Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
      connect(FiveBus.p, pwLine2.n)
        annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
      connect(pwLine2.p, ThreeBus.p)
        annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
      connect(pwLine3.p, breaker.r)
        annotation (Line(points={{-155,-16},{-182,-16}}, color={0,0,255}));
      connect(breaker1.Trigger, breaker.Trigger) annotation (Line(points={{-106,-4},
              {-106,4},{-192,4},{-192,-4}},          color={255,0,255}));
      connect(ThreeBus.p, breaker1.r) annotation (Line(points={{-72,-92},{-72,
              -16},{-96,-16}},                    color={0,0,255}));
      connect(generatorTGOV.pwPin, TwoBus.p) annotation (Line(points={{-195.64,
              -92.2},{-178.44,-92.2},{-178.44,-92},{-164,-92}},     color={0,
              0,255}));
      connect(infiniteBus.p, OneBus.p)
        annotation (Line(points={{-278,-16},{-234,-16}},
                                                       color={0,0,255}));
      connect(pwLine1.n, breaker1.r) annotation (Line(points={{-157,-52},{-86,
              -52},{-86,-16},{-96,-16}}, color={0,0,255}));
      connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
              -16},{-226,-52},{-175,-52}},
                                        color={0,0,255}));
      connect(pwFault.p, pwLine3.n) annotation (Line(points={{-117,-40},{-126,-40},{
              -126,-16},{-137,-16}}, color={0,0,255}));
      connect(breaker1.s, pwLine3.n)
        annotation (Line(points={{-116,-16},{-137,-16}}, color={0,0,255}));
      connect(breaker.s, pwLine1.p) annotation (Line(points={{-202,-16},{-226,
              -16},{-226,-52},{-175,-52}}, color={0,0,255}));
      connect(TwoBus.p, twoWindingTransformer.p)
        annotation (Line(points={{-164,-92},{-119,-92}}, color={0,0,255}));
      connect(ThreeBus.p, twoWindingTransformer.n)
        annotation (Line(points={{-72,-92},{-105,-92}}, color={0,0,255}));
      connect(load.p, FiveBus.p) annotation (Line(points={{86,-104},{86,-92},
              {28,-92}}, color={0,0,255}));
      connect(realExpression.y, recordReferenceRelay.u) annotation (Line(points=
             {{-181,22},{-168,22},{-168,23},{-155.333,23}}, color={0,0,127}));
      connect(recordReferenceRelay.TripSingal, breaker.Trigger) annotation (
          Line(points={{-129.333,23},{-106,23},{-106,4},{-192,4},{-192,-4}},
            color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                -140},{140,80}})),      Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
    end AIOSRecordNoMotorRelay;
  end TestSystems;

  package Data
    package Records
      record VoltagePFData
          //Infinite Bus
         parameter Real InfiniteBusV_0;
         parameter Real InfiniteBusangle_0;
          //Breaker
         parameter Boolean BreakerenableTrigger;
          //TransformerB3B4
         parameter Real TransformerB3B4m;
          //Generator
         parameter Real GeneratorV_0;
         parameter Real Generatorangle_0;
          //PQ Load
         parameter Real PQLoadV_0;
         parameter Real PQLoadangle_0;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end VoltagePFData;

      record PowerPFData
        //Infinite Bus
        parameter Real InfiniteBusP_0;
        parameter Real InfiniteBusQ_0;
        //Generator
        parameter Real GeneratorP_0;
        parameter Real GeneratorQ_0;
        //PQLoad
        parameter Real PQLoadP_0;
        parameter Real PQLoadQ_0;
        //Motor
        parameter Real MotorP_0;
        parameter Real MotorQ_0;
        //ShuntCapacitor
        parameter Real ShuntCapacitorQnom;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PowerPFData;

      record PowerFlow
        extends Modelica.Icons.Record;
        replaceable record Voltage =
            OCRAIOSPSSE.Data.Records.VoltagePFData constrainedby
          OCRAIOSPSSE.Data.Records.VoltagePFData
        annotation(choicesAllMatching);
        Voltage voltage;

        replaceable record Power =
            OCRAIOSPSSE.Data.Records.PowerPFData constrainedby
          OCRAIOSPSSE.Data.Records.PowerPFData
        annotation (choicesAllMatching);
        Power power;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PowerFlow;
    end Records;

    package PowerData

      record PPF1
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=-350,
          InfiniteBusQ_0=-16,
          GeneratorP_0=450,
          GeneratorQ_0=99,
          PQLoadP_0=100,
          PQLoadQ_0=20,
          MotorP_0=0,
          MotorQ_0=0,
          ShuntCapacitorQnom=0);
      end PPF1;

      record PPF2
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=50,
          InfiniteBusQ_0=60,
          GeneratorP_0=350,
          GeneratorQ_0=47,
          PQLoadP_0=400,
          PQLoadQ_0=80,
          MotorP_0=0,
          MotorQ_0=0,
          ShuntCapacitorQnom=0);
      end PPF2;

      record PPF3
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=150,
          InfiniteBusQ_0=66,
          GeneratorP_0=350,
          GeneratorQ_0=50,
          PQLoadP_0=500,
          PQLoadQ_0=80,
          MotorP_0=0,
          MotorQ_0=0,
          ShuntCapacitorQnom=0);
      end PPF3;

      record PPF4
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=400,
          InfiniteBusQ_0=107,
          GeneratorP_0=300,
          GeneratorQ_0=213,
          PQLoadP_0=500,
          PQLoadQ_0=80,
          MotorP_0=0.01333,
          MotorQ_0=0,
          ShuntCapacitorQnom=117);
      end PPF4;

      record PPF5
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=9000,
          InfiniteBusQ_0=236,
          GeneratorP_0=300,
          GeneratorQ_0=37,
          PQLoadP_0=1200,
          PQLoadQ_0=0,
          MotorP_0=0,
          MotorQ_0=0,
          ShuntCapacitorQnom=0);
      end PPF5;

      record PPF6
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=1200,
          InfiniteBusQ_0=434,
          GeneratorP_0=300,
          GeneratorQ_0=213,
          PQLoadP_0=1500,
          PQLoadQ_0=150,
          MotorP_0=0,
          MotorQ_0=0,
          ShuntCapacitorQnom=0);
      end PPF6;

      record PPF7
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=1050,
          InfiniteBusQ_0=374,
          GeneratorP_0=450,
          GeneratorQ_0=198,
          PQLoadP_0=1500,
          PQLoadQ_0=150,
          MotorP_0=0,
          MotorQ_0=0,
          ShuntCapacitorQnom=0);
      end PPF7;

      record PPF8
      extends OCRAIOSPSSE.Data.Records.PowerPFData(
          InfiniteBusP_0=1200,
          InfiniteBusQ_0=454,
          GeneratorP_0=300,
          GeneratorQ_0=177,
          PQLoadP_0=900,
          PQLoadQ_0=50,
          MotorP_0=0.01333,
          MotorQ_0=0,
          ShuntCapacitorQnom=117);
      end PPF8;

    end PowerData;

    package VoltageData

        record VPF1
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.06,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.04,
          Generatorangle_0=8.8,
          PQLoadV_0=1.0675,
          PQLoadangle_0=4.7);
        end VPF1;

        record VPF2
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.06,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.01,
          Generatorangle_0=2.5,
          PQLoadV_0=1.0411,
          PQLoadangle_0=-1.6);
        end VPF2;

        record VPF3
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.06,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.01,
          Generatorangle_0=1.0,
          PQLoadV_0=1.0404,
          PQLoadangle_0=-3.2);
        end VPF3;

        record VPF4
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.04,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.00,
          Generatorangle_0=-9.4,
          PQLoadV_0=1.0024,
          PQLoadangle_0=-13.3);
        end VPF4;

        record VPF5
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.08,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.01,
          Generatorangle_0=-10,
          PQLoadV_0=1.0445,
          PQLoadangle_0=-15.2);
        end VPF5;

        record VPF6
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.08,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.01,
          Generatorangle_0=-14.8,
          PQLoadV_0=1.0089,
          PQLoadangle_0=-20.9);
        end VPF6;

        record VPF7
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.08,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1.01,
          Generatorangle_0=-11.1,
          PQLoadV_0=1.0129,
          PQLoadangle_0=-18.6);
        end VPF7;

        record VPF8
        extends OCRAIOSPSSE.Data.Records.VoltagePFData(
          InfiniteBusV_0=1.08,
          InfiniteBusangle_0=0.0,
          BreakerenableTrigger=false,
          TransformerB3B4m=9999,
          GeneratorV_0=1,
          Generatorangle_0=-14.8,
          PQLoadV_0=1.0091,
          PQLoadangle_0=-19.7);
        end VPF8;

    end VoltageData;

    package SystemData
      record SystemData

        record PF1 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF1, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF1)
                                                             "Power FLow 1";
        record PF2 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF2, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF2)
                                                             "Power FLow 2";
        record PF3 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF3, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF3)
                                                             "Power FLow 3";

        record PF4 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF4, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF4)
                                                             "Power FLow 4";
        record PF5 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF5, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF5)
                                                             "Power FLow 5";

        record PF6 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF6, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF6)
                                                             "Power FLow 6";

        record PF7 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF7, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF7)
                                                             "Power FLow 7";

        record PF8 =
        OCRAIOSPSSE.Data.Records.PowerFlow (redeclare replaceable record
              Voltage = OCRAIOSPSSE.Data.VoltageData.VPF8, redeclare
              replaceable record Power = OCRAIOSPSSE.Data.PowerData.PPF8)
                                                             "Power FLow 8";
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end SystemData;
    end SystemData;
  end Data;
  annotation (uses(iPSL(version="1.1.0"),
      Modelica(version="3.2.2"),
      Complex(version="3.2.2"),
      Tutorial(version="1"),
      OpenIPSL(version="2.0.0-dev"),
      Modelica_Synchronous(version="0.93.0")));
end OCRAIOSPSSE;

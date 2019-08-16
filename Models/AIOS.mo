within ;
package RelayCoordination
  package Components
    model VoltageSource
      extends OpenIPSL.Electrical.Essentials.pfComponent;
      Modelica.SIunits.Conversions.NonSIunits.Angle_deg angle(start=angle_0)
        "Bus voltage angle";
      Real V(start=V_0) "Bus voltage magnitude (pu)";
      Real P "Active Power absorbed by the Infinite bus (MW)";
      Real Q "Reactive Power absorbed by the Infinite bus (MVAr)";
      parameter Boolean displayPF=true "Display voltage values:" annotation (Dialog(
          group="Visualisation",
          __Dymola_compact=true,
          __Dymola_descriptionLabel=true), choices(checkBox=true));
      OpenIPSL.Interfaces.PwPin p(vr(start=V_0*cos(angle_0*Modelica.Constants.pi/180)), vi(start=V_0*
              sin(angle_0*Modelica.Constants.pi/180)))
        annotation (Placement(
          visible=true,
          transformation(
            origin={110,0},
            extent={{-10.0,-10.0},{10.0,10.0}},
            rotation=0),
          iconTransformation(
            origin={110,0},
            extent={{-10,-10},{10,10}},
            rotation=0)));
      Modelica.Blocks.Interfaces.RealInput u1 "Real Part"
        annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
      Modelica.Blocks.Interfaces.RealInput u2 "Imaginary Part"
        annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
    equation
      p.vr = u1;
      p.vi = u2;
      V = sqrt(p.vr^2 + p.vi^2);
      angle = atan2(p.vi, p.vr)*180/Modelica.Constants.pi;
      P = -(p.vr*p.ir + p.vi*p.ii)*S_b;
      Q = -(p.vr*p.ii - p.vi*p.ir)*S_b;
      annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
            graphics={                 Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),Line(
              points={{-20,20},{-50,40},{-80,0}},
              color={0,0,0},
              thickness=0.5),                Line(
              points={{20,-20},{50,-40},{80,0}},
              color={0,0,0},
              thickness=0.5),Text(
              extent={{-40,20},{40,-20}},
              lineColor={0,0,0},
              textString="%name")}));
    end VoltageSource;

    package RelayPackOCR
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

        model TransmissionLine
          Modelica.Electrical.Analog.Basic.Ground ground
            annotation (Placement(transformation(extent={{-64,-52},{-44,-32}})));
          Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.050)
            annotation (Placement(transformation(extent={{-34,28},{-14,48}})));
          Modelica.Electrical.Analog.Basic.Capacitor capacitor(C=4.47092e-9)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=-90,
                origin={-54,10})));
          Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.0012944)
            annotation (Placement(transformation(extent={{-4,28},{16,48}})));
          Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
            annotation (Placement(transformation(extent={{-116,30},{-96,50}})));
          Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
            annotation (Placement(transformation(extent={{94,30},{114,50}})));
          Modelica.Electrical.Analog.Basic.Capacitor capacitor1(C=4.47092e-9)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=-90,
                origin={46,10})));
          Modelica.Electrical.Analog.Basic.Ground ground1
            annotation (Placement(transformation(extent={{36,-50},{56,-30}})));
        equation
          connect(inductor.p, resistor.n)
            annotation (Line(points={{-4,38},{-14,38}}, color={0,0,255}));
          connect(pin_p, resistor.p)
            annotation (Line(points={{-106,40},{-70,40},{-70,38},{-34,38}},
                                                          color={0,0,255}));
          connect(inductor.n, pin_n)
            annotation (Line(points={{16,38},{60,38},{60,40},{104,40}},
                                                        color={0,0,255}));
          connect(capacitor.p, resistor.p) annotation (Line(points={{-54,20},{-54,
                  38},{-34,38}}, color={0,0,255}));
          connect(capacitor1.p, pin_n)
            annotation (Line(points={{46,20},{46,40},{104,40}}, color={0,0,255}));
          connect(capacitor.n, ground.p)
            annotation (Line(points={{-54,0},{-54,-32}}, color={0,0,255}));
          connect(ground1.p, capacitor1.n) annotation (Line(points={{46,-30},{46,0}},
                                   color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                         Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end TransmissionLine;

        model CurrentTransformer
          Modelica.Blocks.Interfaces.BooleanInput u
            annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
          Modelica.Blocks.Interfaces.RealOutput y
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Modelica.Blocks.Math.Division division
            annotation (Placement(transformation(extent={{64,-10},{84,10}})));
          Modelica.Blocks.Logical.Switch switch1
            annotation (Placement(transformation(extent={{0,-10},{20,10}})));
          Modelica.Blocks.Sources.Constant const1(k=233)
            annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
          Modelica.Blocks.Sources.Constant const(k=0.00001)
            annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
          Modelica.Blocks.Interfaces.RealInput u1
            annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
        equation
          connect(y, division.y)
            annotation (Line(points={{110,0},{85,0}}, color={0,0,127}));
          connect(division.u2, switch1.y) annotation (Line(points={{62,-6},{42,-6},
                  {42,0},{21,0}}, color={0,0,127}));
          connect(switch1.u3, const1.y) annotation (Line(points={{-2,-8},{-14,-8},{
                  -14,-30},{-59,-30}}, color={0,0,127}));
          connect(switch1.u1, const.y) annotation (Line(points={{-2,8},{-24,8},{-24,
                  30},{-59,30}}, color={0,0,127}));
          connect(u, switch1.u2)
            annotation (Line(points={{-120,0},{-2,0}}, color={255,0,255}));
          connect(u1, division.u1) annotation (Line(points={{-120,60},{42,60},{42,6},
                  {62,6}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                         Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end CurrentTransformer;

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
          RelayPackOCR.Components.DiscreteFourier.Components.DiscreteMeanValue discreteMeanValue(
            FreqHz=FreqHz,
            Ts=Ts,
            Vinit=Vinit,
            booleanStep(startTime=1/FreqHz),
            gain1(k=0),
            gain(k=0),
            Delay=0.02) annotation (Placement(transformation(extent={{-166,64},
                    {-130,86}})));
          RelayPackOCR.Components.DiscreteFourier.Components.ElementWiseMult intoreal
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
          RelayPackOCR.Components.DiscreteFourier.Components.ElementWiseMult intoimag
            annotation (Placement(transformation(extent={{-250,38},{-230,58}})));
          parameter Real FreqHz=50 "Constant output value";
          parameter Modelica.SIunits.Time Ts=5e-5 "Sample period of component";
          parameter Real Vinit=1 "Constant output value";
          RelayPackOCR.Components.DiscreteFourier.Components.DiscreteMeanValue discreteMeanValue2(
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

        block ModifiedLimIntegrator
          "Integrator with limited value of the output"
          import Modelica.Blocks.Types.Init;
          parameter Real k(unit="1")=1 "Integrator gain";
          parameter Real outMax(start=1) "Upper limit of output";
          parameter Real outMin=-outMax "Lower limit of output";
          parameter Modelica.Blocks.Types.Init initType=Modelica.Blocks.Types.Init.InitialState
            "Type of initialization (1: no init, 2: steady state, 3/4: initial output)"
            annotation(Evaluate=true, Dialog(group="Initialization"));

          parameter Boolean limitsAtInit = true
            "= false, if limits are ignored during initialization (i.e., der(y)=k*u)"
            annotation(Evaluate=true, Dialog(group="Initialization"));
          parameter Real y_start=0
            "Initial or guess value of output (must be in the limits outMin .. outMax)"
            annotation (Dialog(group="Initialization"));
          parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
            annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
          extends Modelica.Blocks.Interfaces.SISO(y(start=y_start));

        initial equation

          if initType == Init.SteadyState then
             der(y) = 0;
          elseif initType == Init.InitialState or
                 initType == Init.InitialOutput then
            y = y_start;
          end if;
        equation
          if initial() and not limitsAtInit then
             der(y) = k*u;
             assert(y >= outMin - 0.001*abs(outMax-outMin) and y <= outMax + 0.001*abs(outMax-outMin),
                  "LimIntegrator: During initialization the limits have been ignored.\n"
                + "However, the result is that the output y is not within the required limits:\n"
                + "  y = " + String(y) + ", outMin = " + String(outMin) + ", outMax = " + String(outMax));
          elseif strict then
             der(y) = noEvent(if y < outMin and k*u < 0 or y > outMax and k*u > 0 then 0 else k*u);
          else
             der(y) = if y < outMin and k*u < 0 or y > outMax and k*u > 0 then 0 else k*u;
          end if;
          annotation (
            Documentation(info="<html>
<p>
This blocks computes <b>y</b> (element-wise) as <i>integral</i>
of the input <b>u</b> multiplied with the gain <i>k</i>. If the
integral reaches a given upper or lower <i>limit</i> and the
input will drive the integral outside of this bound, the
integration is halted and only restarted if the input drives
the integral away from the bounds.
</p>

<p>
It might be difficult to initialize the integrator in steady state.
This is discussed in the description of package
<a href=\"modelica://Modelica.Blocks.Continuous#info\">Continuous</a>.
</p>

<p>
If parameter <b>limitAtInit</b> = <b>false</b>, the limits of the
integrator are removed from the initialization problem which
leads to a much simpler equation system. After initialization has been
performed, it is checked via an assert whether the output is in the
defined limits. For backward compatibility reasons
<b>limitAtInit</b> = <b>true</b>. In most cases it is best
to use <b>limitAtInit</b> = <b>false</b>.
</p>
</html>"),         Icon(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-100,-100},{100,100}}), graphics={
                Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
                Polygon(
                  points={{-80,90},{-88,68},{-72,68},{-80,90}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
                Polygon(
                  points={{90,-80},{68,-72},{68,-88},{90,-80}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-80,-80},{20,20},{80,20}}, color={0,0,127}),
                Text(
                  extent={{0,-10},{60,-70}},
                  lineColor={192,192,192},
                  textString="I"),
                Text(
                  extent={{-150,-150},{150,-110}},
                  lineColor={0,0,0},
                  textString="k=%k"),
                Line(
                  visible=strict,
                  points={{20,20},{80,20}},
                  color={255,0,0})}),
            Diagram(coordinateSystem(
                preserveAspectRatio=true,
                extent={{-100,-100},{100,100}}), graphics={
                Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,255}),
                Text(
                  extent={{-54,46},{-4,-48}},
                  lineColor={0,0,0},
                  textString="lim"),
                Line(points={{-100,0},{-60,0}}, color={0,0,255}),
                Line(points={{60,0},{100,0}}, color={0,0,255}),
                Text(
                  extent={{-8,60},{60,2}},
                  lineColor={0,0,0},
                  textString="k"),
                Text(
                  extent={{-8,-2},{60,-60}},
                  lineColor={0,0,0},
                  textString="s"),
                Line(points={{4,0},{46,0}})}));
        end ModifiedLimIntegrator;
      end Components;

      model OverCurrrentRelay
        Modelica.Blocks.Math.Add add(k1=-1, k2=1)
          annotation (Placement(transformation(extent={{28,32},{48,52}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-102,30},{-82,50}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-72,30},{-52,50}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-166,8})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-166,34},{-146,54}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-214,-6})));
        Modelica.Blocks.Discrete.ZeroOrderHold ZeroOrderHold(samplePeriod=1.25e-3)
          annotation (Placement(transformation(extent={{-354,44},{-334,64}})));
        Modelica.Blocks.Interfaces.RealOutput FaultPickup
          annotation (Placement(transformation(extent={{100,70},{120,90}})));
        Modelica.Blocks.Interfaces.RealOutput OperationTime
          annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
        Modelica.Blocks.Interfaces.RealOutput RatioInputPickup
          annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
        Tests.TwoDiscreteFourier twoDiscreteFourier
          annotation (Placement(transformation(extent={{-244,24},{-202,56}})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-10,44},{10,64}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-10,10},{10,30}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-402,34},{-362,74}})));
        parameter Real Is=1 "Constant output value";
        Modelica.Blocks.Discrete.TransferFunction lowpassfilter(
          b={-0.006613672333773267832113251785131069482,-0.022179807405151456822789413081409293227,
              -0.027037613615508587078251068192003003787,
              0.018607907581767078875056853348723961972,
              0.130909230314600194544638611660047899932,
              0.260593989352078392318645683189970441163,
              0.318887267371506688551363595252041704953,
              0.260593989352078392318645683189970441163,
              0.130909230314600194544638611660047899932,
              0.018607907581767078875056853348723961972,-0.027037613615508587078251068192003003787,
              -0.022179807405151456822789413081409293227,-0.006613672333773267832113251785131069482},
          samplePeriod=5e-5,
          a={1,0,0,0,0,0,0,0,0,0,0,0,0})
          annotation (Placement(transformation(extent={{-322,44},{-302,64}})));
        Components.CalculatingOperationTime calculatingOperationTime(
          alpha=alpha,
          TMS=TMS,
          C=C,
          ls=ls)
          annotation (Placement(transformation(extent={{-2,-46},{18,-26}})));
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{100,16},{120,36}})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{68,16},{88,36}})));
        parameter Real alpha=0.02 "Constant output value";
        parameter Real TMS=0.5 "Constant output value";
        parameter Real C=0.14 "Constant output value";
        parameter Real ls=1 "Constant output value";
        Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=10e-5)
          annotation (Placement(transformation(extent={{-284,44},{-264,64}})));
      equation
        connect(booleanToReal1.u,greater1. y)
          annotation (Line(points={{-74,40},{-81,40}}, color={255,0,255}));
        connect(greater1.u1,RatioInputPickup)  annotation (Line(points={{-104,40},{
                -126,40},{-126,-80},{110,-80}},
                                          color={0,0,127}));
        connect(product.y,RatioInputPickup)  annotation (Line(points={{-145,44},{
                -126,44},{-126,-80},{110,-80}},
                                          color={0,0,127}));
        connect(booleanToReal1.y, extractingTimeOfFault.u) annotation (Line(points=
                {{-51,40},{-30,40},{-30,54},{-12,54}}, color={0,0,127}));
        connect(timer.u, extractingTimeOfFault.u) annotation (Line(points={{-12,
                24.2},{-30,24.2},{-30,54},{-12,54}}, color={0,0,127}));
        connect(FaultPickup, extractingTimeOfFault.u) annotation (Line(points={{110,
                80},{-30,80},{-30,54},{-12,54}}, color={0,0,127}));
        connect(ZeroOrderHold.u, u)
          annotation (Line(points={{-356,54},{-382,54}}, color={0,0,127}));
        connect(ZeroOrderHold.y, lowpassfilter.u)
          annotation (Line(points={{-333,54},{-324,54}}, color={0,0,127}));
        connect(calculatingOperationTime.CurrentInput, RatioInputPickup)
          annotation (Line(points={{-4,-42},{-26,-42},{-26,-80},{110,-80}},  color=
                {0,0,127}));
        connect(calculatingOperationTime.OperationTime, OperationTime) annotation (
            Line(points={{19,-36},{54,-36},{54,-40},{110,-40}},color={0,0,127}));
        connect(greater2.y, TripSingal) annotation (Line(points={{89,26},{110,26}},
                                     color={255,0,255}));
        connect(add.u1, extractingTimeOfFault.y) annotation (Line(points={{26,48},{
                18,48},{18,54},{11,54}}, color={0,0,127}));
        connect(add.u2, timer.y) annotation (Line(points={{26,36},{18,36},{18,20},{
                11,20}}, color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{49,42},{58,42},{58,26},
                {66,26}}, color={0,0,127}));
        connect(greater2.u2, OperationTime) annotation (Line(points={{66,18},{56,18},
                {56,-36},{54,-36},{54,-40},{110,-40}}, color={0,0,127}));
        connect(calculatingOperationTime.Control, extractingTimeOfFault.u)
          annotation (Line(points={{-4,-30},{-30,-30},{-30,54},{-12,54}},  color={0,
                0,127}));
        connect(twoDiscreteFourier.Magnitude, product.u1) annotation (Line(points={{-199.1,
                49.9},{-181.55,49.9},{-181.55,50},{-168,50}},         color={0,0,
                127}));
        connect(sampler.y, twoDiscreteFourier.Input) annotation (Line(points={{-263,54},
                {-247,54}},                         color={0,0,127}));
        connect(sampler.u, lowpassfilter.y) annotation (Line(points={{-286,54},{
                -301,54}},                     color={0,0,127}));
        connect(const1.y, product.u2) annotation (Line(points={{-203,-6},{-188,-6},
                {-188,38},{-168,38}}, color={0,0,127}));
        connect(const.y, greater1.u2) annotation (Line(points={{-155,8},{-120,8},{
                -120,32},{-104,32}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-580,
                  -260},{280,260}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-580,-260},{280,
                  260}}), graphics={
              Text(
                extent={{-282,36},{-266,24}},
                lineColor={0,0,0},
                textString="(c)
"),           Text(
                extent={{-320,36},{-304,24}},
                lineColor={0,0,0},
                textString="(b)
"),           Text(
                extent={{-366,36},{-350,24}},
                lineColor={0,0,0},
                textString="(a)
"),           Text(
                extent={{-232,40},{-216,28}},
                lineColor={0,0,0},
                textString="(d)
"),           Text(
                extent={{-164,68},{-148,56}},
                lineColor={0,0,0},
                textString="(e)
"),           Text(
                extent={{-8,54},{8,42}},
                lineColor={0,0,0},
                textString="(f)
"),           Text(
                extent={{-8,20},{8,8}},
                lineColor={0,0,0},
                textString="(g)
"),           Text(
                extent={{0,-36},{16,-48}},
                lineColor={0,0,0},
                textString="(h)
")}));
      end OverCurrrentRelay;

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
            RelayPackOCR.Data.Records.AlphaData constrainedby
              RelayPackOCR.Data.Records.AlphaData
            annotation (choicesAllMatching);
            Alpha alpha;
            replaceable record C =
            RelayPackOCR.Data.Records.CData constrainedby
              RelayPackOCR.Data.Records.CData
            annotation (choicesAllMatching);
            C c;
             replaceable record EPS = RelayPackOCR.Data.Records.EPSData
              constrainedby RelayPackOCR.Data.Records.EPSData
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
          record StandardInverseData = RelayPackOCR.Data.Records.RelayData (
              redeclare replaceable record Alpha =
                  RelayPackOCR.Data.AlphaData.SIAlpha,
              redeclare replaceable record C = RelayPackOCR.Data.CData.SIC,
              redeclare replaceable record EPS =
                  RelayPackOCR.Data.EPSData.SIEPS);
          record VeryInverseData = RelayPackOCR.Data.Records.RelayData (
              redeclare replaceable record Alpha =
                  RelayPackOCR.Data.AlphaData.VIAlpha,
              redeclare replaceable record C = RelayPackOCR.Data.CData.VIC,
              redeclare replaceable record EPS =
                  RelayPackOCR.Data.EPSData.VIEPS);
          record ExtremelyInverseData = RelayPackOCR.Data.Records.RelayData (
              redeclare replaceable record Alpha =
                  RelayPackOCR.Data.AlphaData.EIAlpha,
              redeclare replaceable record C = RelayPackOCR.Data.CData.EIC,
              redeclare replaceable record EPS =
                  RelayPackOCR.Data.EPSData.EIEPS);
          record LongInverseData = RelayPackOCR.Data.Records.RelayData (
              redeclare replaceable record Alpha =
                  RelayPackOCR.Data.AlphaData.LIAlpha,
              redeclare replaceable record C = RelayPackOCR.Data.CData.LIC,
              redeclare replaceable record EPS = RelayPackOCR.Data.EPSData.LEPS);
        end RelayData;

        package AlphaData
          record SIAlpha
            extends RelayPackOCR.Data.Records.AlphaData(alpha=0.02);
          end SIAlpha;

          record VIAlpha
            extends RelayPackOCR.Data.Records.AlphaData(alpha=1);
          end VIAlpha;

          record EIAlpha
            extends RelayPackOCR.Data.Records.AlphaData(alpha=2);
          end EIAlpha;

          record LIAlpha
            extends RelayPackOCR.Data.Records.AlphaData(alpha=1);
          end LIAlpha;
        end AlphaData;

        package CData
          record SIC
            extends RelayPackOCR.Data.Records.CData(C=0.14);
          end SIC;

          record VIC
            extends RelayPackOCR.Data.Records.CData(C=13.5);
          end VIC;

          record EIC
            extends RelayPackOCR.Data.Records.CData(C=80);
          end EIC;

          record LIC
            extends RelayPackOCR.Data.Records.CData(C=120);
          end LIC;
        end CData;

        package EPSData
           record SIEPS
            extends RelayPackOCR.Data.Records.EPSData(eps=0.41);
           end SIEPS;

          record VIEPS
            extends RelayPackOCR.Data.Records.EPSData(eps=0.07);
          end VIEPS;

          record EIEPS
            extends RelayPackOCR.Data.Records.EPSData(eps=0.102);
          end EIEPS;

          record LEPS
            extends RelayPackOCR.Data.Records.EPSData(eps=0);
          end LEPS;
        end EPSData;
      end Data;

      model RecordOverCurrrentRelay
        Modelica.Blocks.Math.Add add(k1=-1, k2=1)
          annotation (Placement(transformation(extent={{28,32},{48,52}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-102,30},{-82,50}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-72,30},{-52,50}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-154,-54})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-164,42},{-144,62}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-192,-26})));
        Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(samplePeriod=1.25e-3)
          annotation (Placement(transformation(extent={{-368,44},{-348,64}})));
        Modelica.Blocks.Interfaces.RealOutput FaultPickup
          annotation (Placement(transformation(extent={{100,70},{120,90}})));
        Modelica.Blocks.Interfaces.RealOutput OperationTime
          annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
        Modelica.Blocks.Interfaces.RealOutput RatioInputPickup
          annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-10,44},{10,64}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-10,10},{10,30}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-420,-20},{-380,20}}),
              iconTransformation(extent={{-420,-20},{-380,20}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Discrete.TransferFunction lowpassfilter(
          b={-0.006613672333773267832113251785131069482,-0.022179807405151456822789413081409293227,
              -0.027037613615508587078251068192003003787,
              0.018607907581767078875056853348723961972,
              0.130909230314600194544638611660047899932,
              0.260593989352078392318645683189970441163,
              0.318887267371506688551363595252041704953,
              0.260593989352078392318645683189970441163,
              0.130909230314600194544638611660047899932,
              0.018607907581767078875056853348723961972,-0.027037613615508587078251068192003003787,
              -0.022179807405151456822789413081409293227,-0.006613672333773267832113251785131069482},
          samplePeriod=5e-5,
          a={1,0,0,0,0,0,0,0,0,0,0,0,0})
          annotation (Placement(transformation(extent={{-326,44},{-306,64}})));
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{98,22},{118,42}})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{66,24},{86,44}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=2.5e-5)
          annotation (Placement(transformation(extent={{-288,44},{-268,64}})));
        Components.CalculatingOperationTime calculatingOperationTime(k=0.041)
          annotation (Placement(transformation(extent={{-10,-48},{10,-28}})));
        parameter Real k=eps "Constant output value";
        RelayPack.Tests.TwoDiscreteFourier twoDiscreteFourier
          annotation (Placement(transformation(extent={{-288,10},{-170,92}})));
      equation
        connect(booleanToReal1.u,greater1. y)
          annotation (Line(points={{-74,40},{-81,40}}, color={255,0,255}));
        connect(greater1.u1,RatioInputPickup)  annotation (Line(points={{-104,40},{
                -126,40},{-126,-80},{110,-80}},
                                          color={0,0,127}));
        connect(const1.y,product. u2)
          annotation (Line(points={{-192,-15},{-192,46},{-166,46}},color={0,0,127}));
        connect(product.y,RatioInputPickup)  annotation (Line(points={{-143,52},{
                -126,52},{-126,-80},{110,-80}},
                                          color={0,0,127}));
        connect(booleanToReal1.y, extractingTimeOfFault.u) annotation (Line(points=
                {{-51,40},{-30,40},{-30,54},{-12,54}}, color={0,0,127}));
        connect(timer.u, extractingTimeOfFault.u) annotation (Line(points={{-12,
                24.2},{-30,24.2},{-30,54},{-12,54}}, color={0,0,127}));
        connect(FaultPickup, extractingTimeOfFault.u) annotation (Line(points={{110,
                80},{-30,80},{-30,54},{-12,54}}, color={0,0,127}));
        connect(zeroOrderHold.u, u)
          annotation (Line(points={{-370,54},{-386,54},{-386,0},{-400,0}},
                                                         color={0,0,127}));
        connect(zeroOrderHold.y, lowpassfilter.u) annotation (Line(points={{-347,54},
                {-328,54}},                     color={0,0,127}));
        connect(greater2.y, TripSingal) annotation (Line(points={{87,34},{94.5,34},
                {94.5,32},{108,32}}, color={255,0,255}));
        connect(add.u1, extractingTimeOfFault.y) annotation (Line(points={{26,48},{
                18,48},{18,54},{11,54}}, color={0,0,127}));
        connect(add.u2, timer.y) annotation (Line(points={{26,36},{18,36},{18,20},{
                11,20}}, color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{49,42},{58,42},{58,34},
                {64,34}}, color={0,0,127}));
        connect(lowpassfilter.y, sampler.u) annotation (Line(points={{-305,54},{
                -290,54}},                     color={0,0,127}));
        connect(greater1.u2, const.y) annotation (Line(points={{-104,32},{-154,32},
                {-154,-43}}, color={0,0,127}));
        connect(calculatingOperationTime.Control, extractingTimeOfFault.u)
          annotation (Line(points={{-12,-32},{-40,-32},{-40,16},{-30,16},{-30,54},{
                -12,54}}, color={0,0,127}));
        connect(calculatingOperationTime.CurrentInput, RatioInputPickup)
          annotation (Line(points={{-12,-44},{-40,-44},{-40,-80},{110,-80}}, color=
                {0,0,127}));
        connect(calculatingOperationTime.OperationTime, OperationTime) annotation (
            Line(points={{11,-38},{52,-38},{52,-40},{110,-40}}, color={0,0,127}));
        connect(greater2.u2, OperationTime) annotation (Line(points={{64,26},{52,26},
                {52,-40},{110,-40}}, color={0,0,127}));
        connect(sampler.y, twoDiscreteFourier.Input) annotation (Line(points={{
                -267,54},{-260,54},{-260,56},{-251.8,56}}, color={0,0,127}));
        connect(product.u1, twoDiscreteFourier.Magnitude) annotation (Line(
              points={{-166,58},{-188,58},{-188,56},{-210.6,56}}, color={0,0,
                127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-380,
                  -100},{100,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},{100,
                  100}})));
      end RecordOverCurrrentRelay;

      model ElectricSystemRecordOverCurrrentRelay
        Modelica.Blocks.Math.Add add(k1=-1, k2=1)
          annotation (Placement(transformation(extent={{36,38},{56,58}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-102,28},{-82,48}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-72,30},{-52,50}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-178,-50})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-160,42},{-140,62}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-210,0})));
        Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(samplePeriod=1.25e-3)
          annotation (Placement(transformation(extent={{-368,44},{-348,64}})));
        Tests.TwoDiscreteFourier twoDiscreteFourier
          annotation (Placement(transformation(extent={{-254,32},{-194,84}})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-10,44},{10,64}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-12,-4},{8,16}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-420,-20},{-380,20}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Discrete.TransferFunction lowpassfilter(
          b={-0.006613672333773267832113251785131069482,-0.022179807405151456822789413081409293227,
              -0.027037613615508587078251068192003003787,
              0.018607907581767078875056853348723961972,
              0.130909230314600194544638611660047899932,
              0.260593989352078392318645683189970441163,
              0.318887267371506688551363595252041704953,
              0.260593989352078392318645683189970441163,
              0.130909230314600194544638611660047899932,
              0.018607907581767078875056853348723961972,-0.027037613615508587078251068192003003787,
              -0.022179807405151456822789413081409293227,-0.006613672333773267832113251785131069482},
          samplePeriod=5e-5,
          a={1,0,0,0,0,0,0,0,0,0,0,0,0})
          annotation (Placement(transformation(extent={{-326,44},{-306,64}})));
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{120,-10},{140,10}})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{78,-10},{98,10}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=2.5e-5)
          annotation (Placement(transformation(extent={{-288,44},{-268,64}})));
        Components.CalculatingOperationTime calculatingOperationTime(
          alpha=0.02,
          C=0.14,
          k=0.102)
          annotation (Placement(transformation(extent={{-12,-46},{8,-26}})));
        parameter Real k= 0.041 "Constant output value";
      equation
      //   if greater2.y then
      //      TripSignal = greater2.y;
      //   else
      //     TripSignal = greater2.y-1;
      //   end if;
        connect(booleanToReal1.u,greater1. y)
          annotation (Line(points={{-74,40},{-78,40},{-78,38},{-81,38}},
                                                       color={255,0,255}));
        connect(booleanToReal1.y, extractingTimeOfFault.u) annotation (Line(points=
                {{-51,40},{-30,40},{-30,54},{-12,54}}, color={0,0,127}));
        connect(timer.u, extractingTimeOfFault.u) annotation (Line(points={{-14,
                10.2},{-30,10.2},{-30,54},{-12,54}}, color={0,0,127}));
        connect(zeroOrderHold.y, lowpassfilter.u) annotation (Line(points={{-347,54},
                {-328,54}},                     color={0,0,127}));
        connect(add.u1, extractingTimeOfFault.y) annotation (Line(points={{34,54},{
                11,54}},                 color={0,0,127}));
        connect(add.u2, timer.y) annotation (Line(points={{34,42},{18,42},{18,6},{9,
                6}},     color={0,0,127}));
        connect(lowpassfilter.y, sampler.u) annotation (Line(points={{-305,54},{
                -290,54}},                     color={0,0,127}));
        connect(sampler.y, twoDiscreteFourier.Input) annotation (Line(points={{-267,54},
                {-256,54},{-256,61.1707},{-235.593,61.1707}},
                                                    color={0,0,127}));
        connect(twoDiscreteFourier.Magnitude, product.u1) annotation (Line(points={{
                -214.644,61.1707},{-181.55,61.1707},{-181.55,58},{-162,58}},
                                                                      color={0,0,
                127}));
        connect(calculatingOperationTime.OperationTime, greater2.u2) annotation (
            Line(points={{9,-36},{36,-36},{36,-8},{76,-8}},  color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{57,48},{62,48},{62,0},
                {76,0}}, color={0,0,127}));
        connect(product.y, greater1.u1) annotation (Line(points={{-139,52},{-118,52},
                {-118,38},{-104,38}}, color={0,0,127}));
        connect(calculatingOperationTime.CurrentInput, greater1.u1) annotation (
            Line(points={{-14,-42},{-118,-42},{-118,38},{-104,38}}, color={0,0,127}));
        connect(TripSingal, greater2.y)
          annotation (Line(points={{130,0},{99,0}}, color={255,0,255}));
        connect(TripSingal, TripSingal)
          annotation (Line(points={{130,0},{130,0}}, color={255,0,255}));
        connect(zeroOrderHold.u, u) annotation (Line(points={{-370,54},{-376,54},{
                -376,0},{-400,0}}, color={0,0,127}));
        connect(calculatingOperationTime.Control, extractingTimeOfFault.u)
          annotation (Line(points={{-14,-30},{-30,-30},{-30,54},{-12,54}}, color={0,
                0,127}));
        connect(const1.y, product.u2) annotation (Line(points={{-199,0},{-182,0},{
                -182,46},{-162,46}}, color={0,0,127}));
        connect(const.y, greater1.u2) annotation (Line(points={{-167,-50},{-136,-50},
                {-136,30},{-104,30}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-380,
                  -100},{120,100}}), graphics={Rectangle(extent={{-380,100},{120,
                    -100}}, lineColor={28,108,200})}),                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},{120,
                  100}}), graphics={
              Text(
                extent={{-370,40},{-350,28}},
                lineColor={0,0,0},
                textString="(a)"),
              Text(
                extent={{-326,40},{-306,28}},
                lineColor={0,0,0},
                textString="(b)"),
              Text(
                extent={{-288,40},{-268,28}},
                lineColor={0,0,0},
                textString="(c)"),
              Text(
                extent={{-234,58},{-214,46}},
                lineColor={0,0,0},
                textString="(d)"),
              Text(
                extent={{-164,84},{-144,72}},
                lineColor={0,0,0},
                textString="(e)"),
              Text(
                extent={{-10,60},{10,48}},
                lineColor={0,0,0},
                textString="(f)"),
              Text(
                extent={{-12,12},{8,0}},
                lineColor={0,0,0},
                textString="(g)"),
              Text(
                extent={{-12,-30},{8,-42}},
                lineColor={0,0,0},
                textString="(h)")}));
      end ElectricSystemRecordOverCurrrentRelay;

      model LatchElectricSystemRecordOverCurrrentRelay
        Modelica.Blocks.Math.Add add(k1=-1, k2=1)
          annotation (Placement(transformation(extent={{36,38},{56,58}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-102,28},{-82,48}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-72,30},{-52,50}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-178,-50})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-160,42},{-140,62}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-210,0})));
        Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(samplePeriod=1.25e-3)
          annotation (Placement(transformation(extent={{-368,44},{-348,64}})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-10,44},{10,64}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-12,-4},{8,16}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-420,-20},{-380,20}}),
              iconTransformation(extent={{-420,-20},{-380,20}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Discrete.TransferFunction lowpassfilter(
          b={-0.006613672333773267832113251785131069482,-0.022179807405151456822789413081409293227,
              -0.027037613615508587078251068192003003787,
              0.018607907581767078875056853348723961972,
              0.130909230314600194544638611660047899932,
              0.260593989352078392318645683189970441163,
              0.318887267371506688551363595252041704953,
              0.260593989352078392318645683189970441163,
              0.130909230314600194544638611660047899932,
              0.018607907581767078875056853348723961972,-0.027037613615508587078251068192003003787,
              -0.022179807405151456822789413081409293227,-0.006613672333773267832113251785131069482},
          samplePeriod=5e-5,
          a={1,0,0,0,0,0,0,0,0,0,0,0,0})
          annotation (Placement(transformation(extent={{-326,44},{-306,64}})));
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{120,-10},{140,10}}),
              iconTransformation(extent={{120,-10},{140,10}})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{72,-8},{92,12}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=2.5e-5)
          annotation (Placement(transformation(extent={{-288,44},{-268,64}})));
        Components.CalculatingOperationTime calculatingOperationTime(
          alpha=0.02,
          C=0.14,
          k=0.102)
          annotation (Placement(transformation(extent={{-12,-46},{8,-26}})));
        parameter Real k= 0.041 "Constant output value";
        Components.TwoDiscreteFourier twoDiscreteFourier
          annotation (Placement(transformation(extent={{-282,18},{-164,100}})));
        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop
          annotation (Placement(transformation(extent={{134,-16},{154,4}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
          annotation (Placement(transformation(extent={{74,-42},{94,-22}})));
      equation
      //   if greater2.y then
      //      TripSignal = greater2.y;
      //   else
      //     TripSignal = greater2.y-1;
      //   end if;
        connect(booleanToReal1.u,greater1. y)
          annotation (Line(points={{-74,40},{-78,40},{-78,38},{-81,38}},
                                                       color={255,0,255}));
        connect(booleanToReal1.y, extractingTimeOfFault.u) annotation (Line(points=
                {{-51,40},{-30,40},{-30,54},{-12,54}}, color={0,0,127}));
        connect(timer.u, extractingTimeOfFault.u) annotation (Line(points={{-14,
                10.2},{-30,10.2},{-30,54},{-12,54}}, color={0,0,127}));
        connect(zeroOrderHold.y, lowpassfilter.u) annotation (Line(points={{-347,54},
                {-328,54}},                     color={0,0,127}));
        connect(add.u1, extractingTimeOfFault.y) annotation (Line(points={{34,54},{
                11,54}},                 color={0,0,127}));
        connect(add.u2, timer.y) annotation (Line(points={{34,42},{18,42},{18,6},{9,
                6}},     color={0,0,127}));
        connect(lowpassfilter.y, sampler.u) annotation (Line(points={{-305,54},{
                -290,54}},                     color={0,0,127}));
        connect(calculatingOperationTime.OperationTime, greater2.u2) annotation (
            Line(points={{9,-36},{36,-36},{36,-6},{70,-6}},  color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{57,48},{62,48},{62,2},
                {70,2}}, color={0,0,127}));
        connect(product.y, greater1.u1) annotation (Line(points={{-139,52},{-118,52},
                {-118,38},{-104,38}}, color={0,0,127}));
        connect(calculatingOperationTime.CurrentInput, greater1.u1) annotation (
            Line(points={{-14,-42},{-118,-42},{-118,38},{-104,38}}, color={0,0,127}));
        connect(TripSingal, TripSingal)
          annotation (Line(points={{130,0},{130,0}}, color={255,0,255}));
        connect(zeroOrderHold.u, u) annotation (Line(points={{-370,54},{-386,54},
                {-386,0},{-400,0}},color={0,0,127}));
        connect(calculatingOperationTime.Control, extractingTimeOfFault.u)
          annotation (Line(points={{-14,-30},{-30,-30},{-30,54},{-12,54}}, color={0,
                0,127}));
        connect(const1.y, product.u2) annotation (Line(points={{-199,0},{-182,0},{
                -182,46},{-162,46}}, color={0,0,127}));
        connect(const.y, greater1.u2) annotation (Line(points={{-167,-50},{-136,-50},
                {-136,30},{-104,30}}, color={0,0,127}));
        connect(sampler.y, twoDiscreteFourier.Input) annotation (Line(points={{
                -267,54},{-256,54},{-256,64},{-245.8,64}}, color={0,0,127}));
        connect(product.u1, twoDiscreteFourier.Magnitude) annotation (Line(
              points={{-162,58},{-184,58},{-184,64},{-204.6,64}}, color={0,0,
                127}));
        connect(TripSingal, rSFlipFlop.Q)
          annotation (Line(points={{130,0},{155,0}}, color={255,0,255}));
        connect(greater2.y, rSFlipFlop.S) annotation (Line(points={{93,2},{114,
                2},{114,0},{132,0}}, color={255,0,255}));
        connect(rSFlipFlop.R, booleanConstant.y) annotation (Line(points={{132,
                -12},{114,-12},{114,-32},{95,-32}}, color={255,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-380,
                  -100},{120,100}}), graphics={Rectangle(extent={{-380,100},{
                    120,-100}},
                            lineColor={28,108,200})}),                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-380,-100},{120,
                  100}}), graphics={
              Text(
                extent={{-370,40},{-350,28}},
                lineColor={0,0,0},
                textString="(a)"),
              Text(
                extent={{-326,40},{-306,28}},
                lineColor={0,0,0},
                textString="(b)"),
              Text(
                extent={{-288,40},{-268,28}},
                lineColor={0,0,0},
                textString="(c)"),
              Text(
                extent={{-234,58},{-214,46}},
                lineColor={0,0,0},
                textString="(d)"),
              Text(
                extent={{-164,84},{-144,72}},
                lineColor={0,0,0},
                textString="(e)"),
              Text(
                extent={{-10,60},{10,48}},
                lineColor={0,0,0},
                textString="(f)"),
              Text(
                extent={{-12,12},{8,0}},
                lineColor={0,0,0},
                textString="(g)"),
              Text(
                extent={{-12,-30},{8,-42}},
                lineColor={0,0,0},
                textString="(h)")}));
      end LatchElectricSystemRecordOverCurrrentRelay;

      model CoordinationRecordOverCurrrentRelay
        Modelica.Blocks.Math.Add add(k1=-1, k2=1)
          annotation (Placement(transformation(extent={{36,38},{56,58}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-102,28},{-82,48}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-72,30},{-52,50}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-178,-50})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-160,42},{-140,62}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-210,0})));
        Modelica.Blocks.Discrete.ZeroOrderHold zeroOrderHold(samplePeriod=1.25e-3)
          annotation (Placement(transformation(extent={{-368,44},{-348,64}})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-10,44},{10,64}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-12,-4},{8,16}})));
        Modelica.Blocks.Interfaces.RealInput u
          annotation (Placement(transformation(extent={{-536,44},{-516,64}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Discrete.TransferFunction lowpassfilter(
          b={-0.006613672333773267832113251785131069482,-0.022179807405151456822789413081409293227,
              -0.027037613615508587078251068192003003787,
              0.018607907581767078875056853348723961972,
              0.130909230314600194544638611660047899932,
              0.260593989352078392318645683189970441163,
              0.318887267371506688551363595252041704953,
              0.260593989352078392318645683189970441163,
              0.130909230314600194544638611660047899932,
              0.018607907581767078875056853348723961972,-0.027037613615508587078251068192003003787,
              -0.022179807405151456822789413081409293227,-0.006613672333773267832113251785131069482},
          samplePeriod=5e-5,
          a={1,0,0,0,0,0,0,0,0,0,0,0,0})
          annotation (Placement(transformation(extent={{-326,44},{-306,64}})));
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{120,-10},{140,10}})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{78,-10},{98,10}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Modelica.Blocks.Discrete.Sampler sampler(samplePeriod=2.5e-5)
          annotation (Placement(transformation(extent={{-288,44},{-268,64}})));
        Components.CalculatingOperationTime calculatingOperationTime(
          alpha=0.02,
          C=0.14,
          k=0.102)
          annotation (Placement(transformation(extent={{-12,-46},{8,-26}})));
        parameter Real k= 0.041 "Constant output value";
        Components.TwoDiscreteFourier twoDiscreteFourier
          annotation (Placement(transformation(extent={{-284,18},{-166,100}})));
      equation
      //   if greater2.y then
      //      TripSignal = greater2.y;
      //   else
      //     TripSignal = greater2.y-1;
      //   end if;
        connect(booleanToReal1.u,greater1. y)
          annotation (Line(points={{-74,40},{-78,40},{-78,38},{-81,38}},
                                                       color={255,0,255}));
        connect(booleanToReal1.y, extractingTimeOfFault.u) annotation (Line(points=
                {{-51,40},{-30,40},{-30,54},{-12,54}}, color={0,0,127}));
        connect(timer.u, extractingTimeOfFault.u) annotation (Line(points={{-14,
                10.2},{-30,10.2},{-30,54},{-12,54}}, color={0,0,127}));
        connect(zeroOrderHold.y, lowpassfilter.u) annotation (Line(points={{-347,54},
                {-328,54}},                     color={0,0,127}));
        connect(add.u1, extractingTimeOfFault.y) annotation (Line(points={{34,54},{
                11,54}},                 color={0,0,127}));
        connect(add.u2, timer.y) annotation (Line(points={{34,42},{18,42},{18,6},{9,
                6}},     color={0,0,127}));
        connect(lowpassfilter.y, sampler.u) annotation (Line(points={{-305,54},{
                -290,54}},                     color={0,0,127}));
        connect(calculatingOperationTime.OperationTime, greater2.u2) annotation (
            Line(points={{9,-36},{36,-36},{36,-8},{76,-8}},  color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{57,48},{62,48},{62,0},
                {76,0}}, color={0,0,127}));
        connect(product.y, greater1.u1) annotation (Line(points={{-139,52},{-118,52},
                {-118,38},{-104,38}}, color={0,0,127}));
        connect(calculatingOperationTime.CurrentInput, greater1.u1) annotation (
            Line(points={{-14,-42},{-118,-42},{-118,38},{-104,38}}, color={0,0,127}));
        connect(TripSingal, greater2.y)
          annotation (Line(points={{130,0},{99,0}}, color={255,0,255}));
        connect(TripSingal, TripSingal)
          annotation (Line(points={{130,0},{130,0}}, color={255,0,255}));
        connect(calculatingOperationTime.Control, extractingTimeOfFault.u)
          annotation (Line(points={{-14,-30},{-30,-30},{-30,54},{-12,54}}, color={0,
                0,127}));
        connect(const1.y, product.u2) annotation (Line(points={{-199,0},{-182,0},{
                -182,46},{-162,46}}, color={0,0,127}));
        connect(const.y, greater1.u2) annotation (Line(points={{-167,-50},{-136,-50},
                {-136,30},{-104,30}}, color={0,0,127}));
        connect(product.u1, twoDiscreteFourier.Magnitude) annotation (Line(
              points={{-162,58},{-184,58},{-184,64},{-206.6,64}}, color={0,0,
                127}));
        connect(sampler.y, twoDiscreteFourier.Input) annotation (Line(points={{
                -267,54},{-258,54},{-258,64},{-247.8,64}}, color={0,0,127}));
        connect(zeroOrderHold.u, u)
          annotation (Line(points={{-370,54},{-526,54}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-540,
                  -100},{120,120}}), graphics={Rectangle(extent={{-380,100},{120,
                    -100}}, lineColor={28,108,200})}),                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-540,-100},{
                  120,120}}),
                          graphics={
              Text(
                extent={{-370,40},{-350,28}},
                lineColor={0,0,0},
                textString="(a)"),
              Text(
                extent={{-326,40},{-306,28}},
                lineColor={0,0,0},
                textString="(b)"),
              Text(
                extent={{-288,40},{-268,28}},
                lineColor={0,0,0},
                textString="(c)"),
              Text(
                extent={{-234,58},{-214,46}},
                lineColor={0,0,0},
                textString="(d)"),
              Text(
                extent={{-164,84},{-144,72}},
                lineColor={0,0,0},
                textString="(e)"),
              Text(
                extent={{-10,60},{10,48}},
                lineColor={0,0,0},
                textString="(f)"),
              Text(
                extent={{-12,12},{8,0}},
                lineColor={0,0,0},
                textString="(g)"),
              Text(
                extent={{-12,-30},{8,-42}},
                lineColor={0,0,0},
                textString="(h)")}));
      end CoordinationRecordOverCurrrentRelay;

      model LatchPowerToRealRecordOverCurrrentRelay
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
              rotation=90,
              origin={-196,110})));
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
        Complex vu(re=u.vr, im=u.vi);
        Complex vy(re=y.vr, im=y.vi);
        Complex iu(re=u.ir, im=u.ii);
        Complex iy(re=y.ir, im=y.ii);
        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop
          annotation (Placement(transformation(extent={{-54,-16},{-34,4}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
          annotation (Placement(transformation(extent={{-98,-46},{-78,-26}})));
        PowerToReal powerToReal
          annotation (Placement(transformation(extent={{-352,72},{-318,96}})));
        OpenIPSL.Interfaces.PwPin u annotation (Placement(transformation(extent={{-378,-6},
                  {-358,14}}),      iconTransformation(extent={{-378,-6},{-358,
                  14}})));
        OpenIPSL.Interfaces.PwPin y
          annotation (Placement(transformation(extent={{-24,-10},{-4,10}})));
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
      equation
        vu = vy;
        iu = -iy;
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
        connect(u, powerToReal.In) annotation (Line(points={{-368,4},{-358,4},{
                -358,83.88},{-354.21,83.88}},
                                            color={0,0,255}));
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
        connect(powerToReal.Out, division.u1) annotation (Line(points={{-315.96,
                84},{-302,84},{-302,32},{-344,32},{-344,8},{-334,8}}, color={0,
                0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-360,
                  -100},{-20,100}}), graphics={Rectangle(extent={{-360,100},{
                    -20,-100}},
                            lineColor={28,108,200}), Text(
                extent={{-290,56},{-96,-46}},
                lineColor={28,108,200},
                textString="Relay")}),                                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-360,-100},{
                  -20,100}}),
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
      end LatchPowerToRealRecordOverCurrrentRelay;

      model NoLatchPowerToRealRecordOverCurrrentRelay
        Modelica.Blocks.Math.Add add(       k2=1, k1=1)
          annotation (Placement(transformation(extent={{-128,30},{-108,50}})));
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
              rotation=90,
              origin={-196,110})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Components.CalculatingOperationTime calculatingOperationTime(
          alpha=0.02,
          C=0.14,
          k=0.102)
          annotation (Placement(transformation(extent={{-174,-56},{-154,-36}})));
        parameter Real k= 0.041 "Constant output value";
        Complex vu(re=u.vr, im=u.vi);
        Complex vy(re=y.vr, im=y.vi);
        Complex iu(re=u.ir, im=u.ii);
        Complex iy(re=y.ir, im=y.ii);
        Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop
          annotation (Placement(transformation(extent={{-54,-16},{-34,4}})));
        Modelica.Blocks.Sources.BooleanConstant booleanConstant(k=false)
          annotation (Placement(transformation(extent={{-98,-46},{-78,-26}})));
        PowerToReal powerToReal
          annotation (Placement(transformation(extent={{-346,-6},{-312,18}})));
        OpenIPSL.Interfaces.PwPin u annotation (Placement(transformation(extent={{-378,-6},
                  {-358,14}}),      iconTransformation(extent={{-378,-6},{-358,
                  14}})));
        OpenIPSL.Interfaces.PwPin y
          annotation (Placement(transformation(extent={{-24,-10},{-4,10}})));
      equation
      //   vu = vy;
      //   iu = -iy;
         y.vr = u.vr;
         y.vi = u.vi;
         y.ir = u.ir;
         y.ii = u.ii;
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
          annotation (Line(points={{-151,44},{-140,44},{-140,46},{-130,46}},
                                                         color={0,0,127}));
        connect(timer.y, add.u2) annotation (Line(points={{-153,-4},{-138,-4},{
                -138,34},{-130,34}}, color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{-107,40},{-100,40},
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
        connect(u, powerToReal.In) annotation (Line(points={{-368,4},{-358,4},{-358,5.88},
                {-348.21,5.88}},            color={0,0,255}));
        connect(calculatingOperationTime.CurrentInput, greater1.u1) annotation (
           Line(points={{-176,-52},{-240,-52},{-240,-78},{-302,-78},{-302,-24},
                {-268,-24},{-268,0},{-254,0}}, color={0,0,127}));
        connect(product.u1, powerToReal.Out)
          annotation (Line(points={{-298,6},{-309.96,6}}, color={0,0,127}));
        connect(greater1.y, booleanToReal1.u)
          annotation (Line(points={{-231,0},{-216,0}}, color={255,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-360,
                  -100},{-20,100}}), graphics={Rectangle(extent={{-360,100},{
                    -20,-100}},
                            lineColor={28,108,200}), Text(
                extent={{-290,56},{-96,-46}},
                lineColor={28,108,200},
                textString="Relay")}),                                 Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-360,-100},{
                  -20,100}}),
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
      end NoLatchPowerToRealRecordOverCurrrentRelay;

      model PowerRecordOverCurrrentRelay
        Modelica.Blocks.Math.Add add(k1=-1, k2=1)
          annotation (Placement(transformation(extent={{28,32},{48,52}})));
        Modelica.Blocks.Logical.Greater greater1
          annotation (Placement(transformation(extent={{-102,30},{-82,50}})));
        Modelica.Blocks.Math.BooleanToReal booleanToReal1
          annotation (Placement(transformation(extent={{-72,30},{-52,50}})));
        Modelica.Blocks.Sources.Constant const(k=Is)
                                                    annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-154,-54})));
        Modelica.Blocks.Math.Product product
          annotation (Placement(transformation(extent={{-164,42},{-144,62}})));
        Modelica.Blocks.Sources.Constant const1(k=1/(sqrt(2))) annotation (Placement(
              transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-192,-26})));
        Modelica.Blocks.Interfaces.RealOutput FaultPickup
          annotation (Placement(transformation(extent={{100,70},{120,90}})));
        Modelica.Blocks.Interfaces.RealOutput OperationTime
          annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
        Modelica.Blocks.Interfaces.RealOutput RatioInputPickup
          annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
        Components.ExtractingTimeOfFault extractingTimeOfFault
          annotation (Placement(transformation(extent={{-10,44},{10,64}})));
        Components.Timer timer
          annotation (Placement(transformation(extent={{-10,10},{10,30}})));
        parameter Real Is=1 "Pick Up Current Value";
        Modelica.Blocks.Interfaces.BooleanOutput TripSingal
          annotation (Placement(transformation(extent={{98,22},{118,42}})));
        Modelica.Blocks.Logical.Greater greater2
          annotation (Placement(transformation(extent={{66,24},{86,44}})));
        parameter Real alpha=0.02 "Alpha Constant Value";
        parameter Real TMS=0.5 "TMS Constant Value";
        parameter Real C=0.14 "C Constant Value";
        //parameter Real ls=1 "Constant output value";
        Components.CalculatingOperationTime calculatingOperationTime(k=0.041)
          annotation (Placement(transformation(extent={{-10,-48},{10,-28}})));
        parameter Real k=eps "Constant output value";
        OpenIPSL.Interfaces.PwPin pwPin
          annotation (Placement(transformation(extent={{-462,-8},{-442,12}})));
        PowerToReal powerToReal
          annotation (Placement(transformation(extent={{-416,-10},{-396,10}})));
      equation
        connect(booleanToReal1.u,greater1. y)
          annotation (Line(points={{-74,40},{-81,40}}, color={255,0,255}));
        connect(greater1.u1,RatioInputPickup)  annotation (Line(points={{-104,40},{
                -126,40},{-126,-80},{110,-80}},
                                          color={0,0,127}));
        connect(const1.y,product. u2)
          annotation (Line(points={{-192,-15},{-192,46},{-166,46}},color={0,0,127}));
        connect(product.y,RatioInputPickup)  annotation (Line(points={{-143,52},{
                -126,52},{-126,-80},{110,-80}},
                                          color={0,0,127}));
        connect(booleanToReal1.y, extractingTimeOfFault.u) annotation (Line(points=
                {{-51,40},{-30,40},{-30,54},{-12,54}}, color={0,0,127}));
        connect(timer.u, extractingTimeOfFault.u) annotation (Line(points={{-12,
                24.2},{-30,24.2},{-30,54},{-12,54}}, color={0,0,127}));
        connect(FaultPickup, extractingTimeOfFault.u) annotation (Line(points={{110,
                80},{-30,80},{-30,54},{-12,54}}, color={0,0,127}));
        connect(greater2.y, TripSingal) annotation (Line(points={{87,34},{94.5,34},
                {94.5,32},{108,32}}, color={255,0,255}));
        connect(add.u1, extractingTimeOfFault.y) annotation (Line(points={{26,48},{
                18,48},{18,54},{11,54}}, color={0,0,127}));
        connect(add.u2, timer.y) annotation (Line(points={{26,36},{18,36},{18,20},{
                11,20}}, color={0,0,127}));
        connect(add.y, greater2.u1) annotation (Line(points={{49,42},{58,42},{58,34},
                {64,34}}, color={0,0,127}));
        connect(greater1.u2, const.y) annotation (Line(points={{-104,32},{-154,32},
                {-154,-43}}, color={0,0,127}));
        connect(calculatingOperationTime.Control, extractingTimeOfFault.u)
          annotation (Line(points={{-12,-32},{-40,-32},{-40,16},{-30,16},{-30,54},{
                -12,54}}, color={0,0,127}));
        connect(calculatingOperationTime.CurrentInput, RatioInputPickup)
          annotation (Line(points={{-12,-44},{-40,-44},{-40,-80},{110,-80}}, color=
                {0,0,127}));
        connect(calculatingOperationTime.OperationTime, OperationTime) annotation (
            Line(points={{11,-38},{52,-38},{52,-40},{110,-40}}, color={0,0,127}));
        connect(greater2.u2, OperationTime) annotation (Line(points={{64,26},{52,26},
                {52,-40},{110,-40}}, color={0,0,127}));
        connect(pwPin, powerToReal.In) annotation (Line(points={{-452,2},{-442,
                2},{-442,-0.1},{-417.3,-0.1}}, color={0,0,255}));
        connect(powerToReal.Out, product.u1) annotation (Line(points={{-394.8,0},
                {-286,0},{-286,58},{-166,58}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-440,
                  -100},{100,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-440,-100},{
                  100,100}})));
      end PowerRecordOverCurrrentRelay;

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
      annotation ();
    end RelayPackOCR;

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

    model PwTrasformerLTC_V
      IEEE_Nordic32_revised.PowerSystems.Electrical.Branches.PwTransformerLTC pwTransformerLTC(
        r0=r0,
        Xn=Xn,
        Snom=Snom,
        Systembase=Systembase)
        annotation (Placement(transformation(extent={{-58,-24},{4,48}})));
      IEEE_Nordic32_revised.PowerSystems.Connectors.PwPin p annotation (
          Placement(transformation(extent={{-82,-8},{-62,12}}),
            iconTransformation(extent={{-92,-16},{-62,12}})));
      IEEE_Nordic32_revised.PowerSystems.Electrical.Sensors.Pw3PhaseVoltage pw3PhaseVoltage(
        RT=0,
        XT=0,
        r=1) annotation (Placement(transformation(extent={{8,-22},{56,34}})));
      IEEE_Nordic32_revised.PowerSystems.Connectors.PwPin n annotation (
          Placement(transformation(extent={{58,2},{78,22}}), iconTransformation(
              extent={{58,-16},{90,14}})));
      IEEE_Nordic32_revised.PowerSystems.Connectors.ImPin rin annotation (
          Placement(transformation(extent={{-96,32},{-76,52}}),
            iconTransformation(
            extent={{13,-14},{-13,14}},
            rotation=180,
            origin={-77,30})));
      IEEE_Nordic32_revised.PowerSystems.Connectors.ImPin u annotation (
          Placement(transformation(extent={{62,24},{82,44}}),
            iconTransformation(extent={{56,14},{82,44}})));
      parameter Real r0;
      parameter Real Xn "Reactance, p.u";
      parameter Real Snom "Nominal apparent power";
      parameter Real Systembase "System power base";
    equation
      connect(pwTransformerLTC.r, rin) annotation (Line(
          points={{-48.7,22.8},{-48.7,29.4},{-86,29.4},{-86,42}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(p, pwTransformerLTC.p) annotation (Line(
          points={{-72,2},{-60,2},{-60,12.72},{-48.7,12.72}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(pw3PhaseVoltage.n, n) annotation (Line(
          points={{44,12.16},{54,12.16},{54,12},{68,12}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(pw3PhaseVoltage.p, pwTransformerLTC.n) annotation (Line(
          points={{20,6},{8,6},{8,12.72},{-5.3,12.72}},
          color={0,0,255},
          smooth=Smooth.None));
      connect(pw3PhaseVoltage.v, u) annotation (Line(
          points={{43.76,-0.16},{54,-0.16},{54,34},{72,34}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-80},{100,60}}), graphics={Text(
              extent={{6,44},{30,32}},
              lineColor={0,0,255},
              textString="1：n")}),                    Icon(
            coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
                {100,60}}), graphics={
                                 Rectangle(extent={{-62,48},{58,-48}}, lineColor={0,
                  0,255}),
            Ellipse(
              extent={{-24,16},{8,-16}},
              lineColor={0,0,255},
              lineThickness=0.5),
            Ellipse(
              extent={{-6,16},{26,-16}},
              lineColor={0,0,255},
              lineThickness=0.5),
            Line(
              points={{-40,0},{-24,0}},
              color={0,0,255},
              thickness=0.5,
              smooth=Smooth.None),
            Line(
              points={{26,0},{42,0}},
              color={0,0,255},
              thickness=0.5,
              smooth=Smooth.None),
            Line(
              points={{-34,-26},{30,28},{20,26}},
              color={0,0,255},
              smooth=Smooth.None,
              thickness=0.5),
            Line(
              points={{30,28},{28,18}},
              color={0,0,255},
              smooth=Smooth.None,
              thickness=0.5)}));
    end PwTrasformerLTC_V;
  end Components;

  package Tests
    model TestTransmissionLine
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
        annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
      OpenIPSL.Electrical.Buses.Bus OneBus1(
        angle_0=0,
        V_b=380,
        V_0=1.06,
        P_0=-175,
        Q_0=-8)
        annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
      OpenIPSL.Electrical.Buses.Bus ThreeBus1(
        V_b=380,
        V_0=1.068,
        P_0=0,
        Q_0=0,
        angle_0=4.9)
        annotation (Placement(transformation(extent={{44,20},{64,40}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.2486,
        G=0,
        B=0)
        annotation (Placement(transformation(extent={{-4,30},{16,50}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.2486,
        G=0,
        B=0)
        annotation (Placement(transformation(extent={{-4,10},{16,30}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        V_0=1.06,
        angle_0=0,
        P_0=-175,
        Q_0=-8)
        annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
        V_b=380,
        V_0=1.06,
        angle_0=4.9,
        P_0=0,
        Q_0=0)
        annotation (Placement(transformation(extent={{88,20},{68,40}})));
    equation
      connect(pwLine2.p, OneBus1.p) annotation (Line(points={{-3,40},{-18,40},{
              -18,30},{-30,30}},
                              color={0,0,255}));
      connect(pwLine3.p, OneBus1.p) annotation (Line(points={{-3,20},{-18,20},{
              -18,30},{-30,30}},
                              color={0,0,255}));
      connect(pwLine2.n, ThreeBus1.p) annotation (Line(points={{15,40},{28,40},
              {28,30},{54,30}}, color={0,0,255}));
      connect(pwLine3.n, ThreeBus1.p) annotation (Line(points={{15,20},{28,20},
              {28,30},{54,30}}, color={0,0,255}));
      connect(infiniteBus.p, OneBus1.p)
        annotation (Line(points={{-60,30},{-30,30}},
                                                   color={0,0,255}));
      connect(infiniteBus1.p, ThreeBus1.p)
        annotation (Line(points={{68,30},{54,30}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestTransmissionLine;

    model TestTransformer
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
        annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        P_0=0,
        V_0=1.0683,
        angle_0=4.9,
        V_b=380)
        annotation (Placement(transformation(extent={{-22,14},{-2,34}})));
      Components.VoltageSource voltageSource(V_b=380)
        annotation (Placement(transformation(extent={{-48,14},{-28,34}})));
      Modelica.Blocks.Sources.Constant VReal(k=1.064395)
        annotation (Placement(transformation(extent={{-98,20},{-78,40}})));
      Modelica.Blocks.Sources.Constant VImag(k=0.0912508)
        annotation (Placement(transformation(extent={{-98,-14},{-78,6}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer twoWindingTransformer(
        V_b=380,
        Vn=380,
        Sn=750,
        rT=0,
        xT=4.8)
        annotation (Placement(transformation(extent={{-2,14},{18,34}})));
      OpenIPSL.Electrical.Buses.Bus FourBus(
        V_b=15,
        V_0=1.0078,
        angle_0=4.9,
        P_0=0,
        Q_0=0)
        annotation (Placement(transformation(extent={{20,14},{40,34}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=15,
        V_0=1.0078,
        angle_0=4.9)
        annotation (Placement(transformation(extent={{84,14},{64,34}})));
    equation
      connect(voltageSource.p, ThreeBus.p)
        annotation (Line(points={{-27,24},{-12,24}}, color={0,0,255}));
      connect(VReal.y, voltageSource.u1) annotation (Line(points={{-77,30},{-62,
              30},{-62,28},{-50,28}}, color={0,0,127}));
      connect(voltageSource.u2, VImag.y) annotation (Line(points={{-50,20},{-56,
              20},{-56,-4},{-77,-4}},
                                    color={0,0,127}));
      connect(ThreeBus.p, twoWindingTransformer.p)
        annotation (Line(points={{-12,24},{-3,24}}, color={0,0,255}));
      connect(twoWindingTransformer.n,FourBus. p)
        annotation (Line(points={{19,24},{30,24}}, color={0,0,255}));
      connect(FourBus.p, infiniteBus.p)
        annotation (Line(points={{30,24},{64,24}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestTransformer;

    model TestSRFlipFlopLatch
      Modelica.Blocks.Sources.BooleanConstant R(k=false)
        annotation (Placement(transformation(extent={{-80,-8},{-60,12}})));
      Modelica.Blocks.Sources.BooleanConstant S(k=false)
        annotation (Placement(transformation(extent={{-82,26},{-62,46}})));
      Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop
        annotation (Placement(transformation(extent={{-34,14},{-14,34}})));
    equation
      connect(S.y, rSFlipFlop.S) annotation (Line(points={{-61,36},{-48,36},{
              -48,30},{-36,30}}, color={255,0,255}));
      connect(R.y, rSFlipFlop.R) annotation (Line(points={{-59,2},{-46,2},{-46,
              18},{-36,18}}, color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestSRFlipFlopLatch;

    model TestLatchPowerToRealFault
      Components.VoltageSource voltageSource
        annotation (Placement(transformation(extent={{-120,-6},{-100,14}})));
      Modelica.Blocks.Math.Add add(k2=+1)
        annotation (Placement(transformation(extent={{-160,8},{-140,28}})));
      Modelica.Blocks.Sources.Constant const1(k=1)
        annotation (Placement(transformation(extent={{-254,14},{-234,34}})));
      Modelica.Blocks.Sources.Step step(startTime=2, height=2)
        annotation (Placement(transformation(extent={{-256,-14},{-236,6}})));
      Modelica.Blocks.Sources.Constant const2(k=0)
        annotation (Placement(transformation(extent={{-256,-46},{-236,-26}})));
      OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={62,4})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
        annotation (Placement(transformation(extent={{-260,80},{-200,100}})));
      OpenIPSL.Electrical.Buses.Bus bus
        annotation (Placement(transformation(extent={{-94,-6},{-74,14}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine(
        R=0.01,
        X=0.1,
        G=0,
        B=0) annotation (Placement(transformation(extent={{-6,6},{14,28}})));
      OpenIPSL.Electrical.Buses.Bus bus1
        annotation (Placement(transformation(extent={{18,-6},{38,14}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0.01,
        X=0.1,
        G=0,
        B=0) annotation (Placement(transformation(extent={{-50,-16},{-30,6}})));
      OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
        annotation (Placement(transformation(extent={{-34,10},{-14,30}})));
      Components.RelayPackOCR.LatchPowerToRealRecordOverCurrrentRelay
        latchPowerToRealRecordOverCurrrentRelay
        annotation (Placement(transformation(extent={{-72,14},{-44,34}})));
    equation
      connect(add.u1, const1.y) annotation (Line(points={{-162,24},{-233,24}},
                                   color={0,0,127}));
      connect(add.u2, step.y) annotation (Line(points={{-162,12},{-216,12},{
              -216,-4},{-235,-4}}, color={0,0,127}));
      connect(add.y, voltageSource.u1) annotation (Line(points={{-139,18},{-128,
              18},{-128,8},{-122,8}}, color={0,0,127}));
      connect(const2.y, voltageSource.u2) annotation (Line(points={{-235,-36},{
              -160,-36},{-160,0},{-122,0}}, color={0,0,127}));
      connect(voltageSource.p, bus.p)
        annotation (Line(points={{-99,4},{-84,4}}, color={0,0,255}));
      connect(bus1.p, lOADPQ.p)
        annotation (Line(points={{28,4},{52,4}}, color={0,0,255}));
      connect(pwLine1.n, bus1.p) annotation (Line(points={{-31,-5},{18,-5},{18,
              4},{28,4}}, color={0,0,255}));
      connect(bus.p, pwLine1.p) annotation (Line(points={{-84,4},{-74,4},{-74,
              -5},{-49,-5}}, color={0,0,255}));
      connect(bus.p, latchPowerToRealRecordOverCurrrentRelay.u) annotation (
          Line(points={{-84,4},{-78,4},{-78,24.4},{-72.6588,24.4}},
                                                           color={0,0,255}));
      connect(breaker.r, pwLine.p) annotation (Line(points={{-14,20},{-4,20},{
              -4,17},{-5,17}}, color={0,0,255}));
      connect(pwLine.n, bus1.p)
        annotation (Line(points={{13,17},{13,4},{28,4}}, color={0,0,255}));
      connect(breaker.s, latchPowerToRealRecordOverCurrrentRelay.y)
        annotation (Line(points={{-34,20},{-38,20},{-38,24},{-43.5059,24}},
                                                          color={0,0,255}));
      connect(latchPowerToRealRecordOverCurrrentRelay.TripSingal, breaker.Trigger)
        annotation (Line(points={{-58.4941,35},{-58.4941,50},{-24,50},{-24,32}},
            color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -260,-100},{100,100}})), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-260,-100},{100,100}})));
    end TestLatchPowerToRealFault;

    model TestGenerator
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
        annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
      OpenIPSL.Electrical.Buses.Bus TwoBus(
        V_b=20,
        V_0=1.04,
        angle_0=8.8)
        annotation (Placement(transformation(extent={{-22,14},{-2,34}})));
      Components.VoltageSource voltageSource(
        V_0=1.04,
        angle_0=8.8,
        P_0=450,
        Q_0=99,
        V_b=20)
        annotation (Placement(transformation(extent={{-48,14},{-28,34}})));
      Modelica.Blocks.Sources.Constant VReal(k=1.02776)
        annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
      Modelica.Blocks.Sources.Constant VImag(k=0.15911)
        annotation (Placement(transformation(extent={{-96,-8},{-76,12}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer twoWindingTransformer(
        rT=0,
        Sn=450,
        Vn=20,
        V_b=20,
        xT=0.08)
        annotation (Placement(transformation(extent={{-2,14},{18,34}})));
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        angle_0=4.9,
        V_b=380,
        V_0=1.0683)
        annotation (Placement(transformation(extent={{20,14},{40,34}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        V_0=1.0675,
        angle_0=4.9)
        annotation (Placement(transformation(extent={{90,14},{70,34}})));
    equation
      connect(voltageSource.p, TwoBus.p)
        annotation (Line(points={{-27,24},{-12,24}}, color={0,0,255}));
      connect(VReal.y,voltageSource. u1) annotation (Line(points={{-75,44},{-62,
              44},{-62,28},{-50,28}}, color={0,0,127}));
      connect(voltageSource.u2,VImag. y) annotation (Line(points={{-50,20},{-56,
              20},{-56,2},{-75,2}}, color={0,0,127}));
      connect(TwoBus.p, twoWindingTransformer.p)
        annotation (Line(points={{-12,24},{-3,24}}, color={0,0,255}));
      connect(twoWindingTransformer.n, ThreeBus.p)
        annotation (Line(points={{19,24},{30,24}}, color={0,0,255}));
      connect(infiniteBus.p, ThreeBus.p)
        annotation (Line(points={{70,24},{30,24}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TestGenerator;
  end Tests;

  model CoordinationAllInOneSystem
    OpenIPSL.Electrical.Buses.Bus OneBus(
      V_b=138,
      V_0=1,
      angle_0=0,
      P_0=-175,
      Q_0=-8)
      annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
    OpenIPSL.Electrical.Buses.Bus ThreeBus(
      V_0=1,
      angle_0=4.9,
      P_0=0,
      Q_0=0,
      V_b=139.08)
      annotation (Placement(transformation(extent={{36,20},{56,40}})));
    OpenIPSL.Electrical.Branches.PwLine pwLine(
      R=1,
      X=1,
      G=1,
      B=1)
      annotation (Placement(transformation(extent={{8,46},{28,66}})));
    OpenIPSL.Electrical.Branches.PwLine pwLine1(
      R=1,
      X=1,
      G=1,
      B=1)
      annotation (Placement(transformation(extent={{8,-2},{28,18}})));
    OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(
      M_b=100,
      D=0,
      V_0=1,
      angle_0=0,
      X_d=0.2,
      P_0=10.01711,
      Q_0=8.006544,
      H=0) annotation (Placement(transformation(extent={{-80,18},{-66,42}})));
    OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeI(
      Sup=0,
      V_0=1.0336,
      angle_0=-0.02173,
      P_0=0.251061717038311,
      Q_0=0.226568616630697) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={186,30})));
    OpenIPSL.Electrical.Buses.Bus FourBus(
      V_b=131.204,
      angle_0=4.9,
      P_0=0,
      Q_0=0) annotation (Placement(transformation(extent={{154,20},{174,40}})));
    OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer twoWindingTransformer(
      Vn=13800,
      xT=0.1,
      rT=0.01,
      V_b=13800)
      annotation (Placement(transformation(extent={{98,20},{118,40}})));
    OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer twoWindingTransformer1(
      Vn=13800,
      xT=0.1,
      rT=0.01,
      V_b=13800)
      annotation (Placement(transformation(extent={{-8,-54},{12,-34}})));
    OpenIPSL.Electrical.Buses.Bus TwoBus(
      V_b=135.396,
      V_0=1,
      P_0=450,
      Q_0=99)
      annotation (Placement(transformation(extent={{-44,-54},{-24,-34}})));
    OpenIPSL.Electrical.Buses.Bus FiveBus(
      V_b=138.97,
      V_0=4.7,
      P_0=100,
      Q_0=20)
      annotation (Placement(transformation(extent={{52,-52},{72,-32}})));
    OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer twoWindingTransformer2(
      Vn=13800,
      xT=0.1,
      rT=0.01,
      V_b=13800)
      annotation (Placement(transformation(extent={{112,-52},{132,-32}})));
    OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={168,-64})));
    inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
      annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
    OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1
      annotation (Placement(transformation(extent={{-82,-54},{-62,-34}})));
    OpenIPSL.Electrical.Buses.Bus FiveBus1(
      V_b=138.97,
      V_0=4.7,
      P_0=100,
      Q_0=20)
      annotation (Placement(transformation(extent={{90,-52},{110,-32}})));
    OpenIPSL.Electrical.Branches.PwLine pwLine2(
      R=1,
      X=1,
      G=1,
      B=1)
      annotation (Placement(transformation(extent={{70,-52},{90,-32}})));
    OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
      annotation (Placement(transformation(extent={{-22,46},{-2,66}})));
    OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
      annotation (Placement(transformation(extent={{-20,18},{0,-2}})));
    OpenIPSL.Electrical.Events.Breaker breaker2(enableTrigger=true)
      annotation (Placement(transformation(extent={{70,20},{90,40}})));
    OpenIPSL.Electrical.Events.Breaker breaker3(enableTrigger=true)
      annotation (Placement(transformation(extent={{130,20},{150,40}})));
    Components.RelayPackOCR.CoordinationRecordOverCurrrentRelay OCR1
      annotation (Placement(transformation(extent={{-32,76},{-16,88}})));
    Components.RelayPackOCR.CoordinationRecordOverCurrrentRelay OCR2
      annotation (Placement(transformation(extent={{-34,-24},{-18,-12}})));
    Components.RelayPackOCR.CoordinationRecordOverCurrrentRelay OCR3
      annotation (Placement(transformation(extent={{60,52},{76,64}})));
    Components.RelayPackOCR.CoordinationRecordOverCurrrentRelay OCR4
      annotation (Placement(transformation(extent={{122,56},{138,68}})));
  equation
    connect(pwLine.n, ThreeBus.p) annotation (Line(points={{27,56},{30,56},{30,
            30},{46,30}}, color={0,0,255}));
    connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{27,8},{30,8},{30,
            30},{46,30}}, color={0,0,255}));
    connect(OneBus.p, gENCLS.p)
      annotation (Line(points={{-50,30},{-66,30}}, color={0,0,255}));
    connect(motorTypeI.p, FourBus.p)
      annotation (Line(points={{176,30},{164,30}}, color={0,0,255}));
    connect(twoWindingTransformer1.n, ThreeBus.p) annotation (Line(points={{13,
            -44},{36,-44},{36,30},{46,30}}, color={0,0,255}));
    connect(twoWindingTransformer1.p, TwoBus.p)
      annotation (Line(points={{-9,-44},{-34,-44}}, color={0,0,255}));
    connect(lOADPQ.p, twoWindingTransformer2.n) annotation (Line(points={{168,
            -54},{168,-42},{133,-42}}, color={0,0,255}));
    connect(order2_1.p, TwoBus.p)
      annotation (Line(points={{-62,-44},{-34,-44}}, color={0,0,255}));
    connect(twoWindingTransformer2.p, FiveBus1.p)
      annotation (Line(points={{111,-42},{100,-42}}, color={0,0,255}));
    connect(FiveBus.p, pwLine2.p)
      annotation (Line(points={{62,-42},{71,-42}}, color={0,0,255}));
    connect(pwLine2.n, FiveBus1.p)
      annotation (Line(points={{89,-42},{100,-42}}, color={0,0,255}));
    connect(pwLin, breaker.r)
      annotation (Line(points={{9,56},{-2,56}}, color={0,0,255}));
    connect(pwLine1.p, breaker1.r)
      annotation (Line(points={{9,8},{0,8}}, color={0,0,255}));
    connect(breaker1.s, OneBus.p) annotation (Line(points={{-20,8},{-34,8},{-34,
            30},{-50,30}}, color={0,0,255}));
    connect(breaker.s, OneBus.p) annotation (Line(points={{-22,56},{-34,56},{
            -34,30},{-50,30}}, color={0,0,255}));
    connect(twoWindingTransformer.p, breaker2.r)
      annotation (Line(points={{97,30},{90,30}}, color={0,0,255}));
    connect(ThreeBus.p, breaker2.s)
      annotation (Line(points={{46,30},{70,30}}, color={0,0,255}));
    connect(twoWindingTransformer.n, breaker3.s)
      annotation (Line(points={{119,30},{130,30}}, color={0,0,255}));
    connect(FourBus.p, breaker3.r)
      annotation (Line(points={{164,30},{150,30}}, color={0,0,255}));
    connect(FiveBus.p, breaker2.s) annotation (Line(points={{62,-42},{50,-42},{
            50,30},{70,30}}, color={0,0,255}));
    connect(OCR1.TripSingal, breaker.Trigger) annotation (Line(points={{
            -15.7576,81.4545},{-12,81.4545},{-12,68}}, color={255,0,255}));
    connect(OCR1.pwPin, OneBus.p) annotation (Line(points={{-31.903,81.3455},{
            -34,81.3455},{-34,30},{-50,30}}, color={0,0,255}));
    connect(OCR2.pwPin, OneBus.p) annotation (Line(points={{-33.903,-18.6545},{
            -33.903,8},{-34,8},{-34,30},{-50,30}}, color={0,0,255}));
    connect(OCR2.TripSingal, breaker1.Trigger) annotation (Line(points={{
            -17.7576,-18.5455},{-10,-18.5455},{-10,-4}}, color={255,0,255}));
    connect(OCR3.TripSingal, breaker2.Trigger) annotation (Line(points={{76.2424,
            57.4545},{80,57.4545},{80,42}},         color={255,0,255}));
    connect(OCR3.pwPin, breaker2.s) annotation (Line(points={{60.097,57.3455},{
            58,57.3455},{58,30},{70,30}}, color={0,0,255}));
    connect(breaker3.Trigger, OCR4.TripSingal) annotation (Line(points={{140,42},
            {140,61.4545},{138.242,61.4545}}, color={255,0,255}));
    connect(OCR4.pwPin, breaker3.s) annotation (Line(points={{122.097,61.3455},
            {114,61.3455},{114,42},{126,42},{126,30},{130,30}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{200,100}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{200,
              100}}), graphics={
          Text(
            extent={{-26,84},{-18,78}},
            lineColor={0,0,0},
            textString="OCR1"),
          Text(
            extent={{-28,-16},{-20,-22}},
            lineColor={0,0,0},
            textString="OCR1b"),
          Text(
            extent={{66,60},{74,54}},
            lineColor={0,0,0},
            textString="OCR2"),
          Text(
            extent={{128,64},{136,58}},
            lineColor={0,0,0},
            textString="OCR3")}));
  end CoordinationAllInOneSystem;

  package AIOS

    package Scenarios
      package LF4
        model LF4AllInOneUnfaulted
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-168,-102},{-148,-82}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1,
            angle_0=-9.4,
            Q_0=213,
            ra=0,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-250,-114},{-206,-70}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-124,-102},{-104,-82}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=1200,
            Q_0=53)
            annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{-8,-102},{12,-82}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0024,
            angle_0=-13.3,
            P_0=500,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-22,-22},{22,22}},
                rotation=90,
                origin={66,-92})));
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.04,
            angle_0=0,
            P_0=800,
            Q_0=214.18834,
            displayPF=true)
            annotation (Placement(transformation(extent={{-274,-8},{-236,16}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-210,-6},{-190,14}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-156,20},{-136,40}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-156,-30},{-136,-10}})));
          OpenIPSL.Electrical.Buses.Bus FourBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-300,
            Q_0=-23)
            annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer2(
            Sn=750,
            V_b=380,
            Vn=380,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1)
            annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
          OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
            Q_0=0,
            Hm=0.6,
            V_b=15,
            Sup=0,
            V_0=1.04,
            angle_0=-0.3,
            Rs=0.031,
            Xs=0.1,
            Rr1=0.05,
            Xr1=0.07,
            Xm=3.20,
            a=0.78,
            P_0=0.01333)
            annotation (Placement(transformation(extent={{62,-10},{42,10}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-40,-102},{-20,-82}})));
          OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Qnom=117, Vbase=15)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={52,-36})));
        equation
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-245.6,
                  -67.8},{-245.6,-64},{-262,-64},{-262,-81},{-254.4,-81}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-245.6,
                  -116.2},{-245.6,-120},{-262,-120},{-262,-103},{-254.4,-103}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-158,-92},{-125,-92}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p)
            annotation (Line(points={{2,-92},{44,-92}},   color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-103,-92},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.p,OneBus. p) annotation (Line(points={{-155,30},{-176,
                  30},{-176,4},{-200,4}},
                                      color={0,0,255}));
          connect(OneBus.p, infiniteBus.p)
            annotation (Line(points={{-200,4},{-236,4}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-137,30},{-122,
                  30},{-122,0},{-72,0},{-72,-92}}, color={0,0,255}));
          connect(FourBus.p, twoWindingTransformer2.n)
            annotation (Line(points={{-6,0},{-25,0}}, color={0,0,255}));
          connect(FourBus.p, motorTypeIII.p)
            annotation (Line(points={{-6,0},{42,0}}, color={0,0,255}));
          connect(twoWindingTransformer2.p, ThreeBus.p) annotation (Line(points={
                  {-47,0},{-72,0},{-72,-92}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine2.p)
            annotation (Line(points={{-72,-92},{-39,-92}}, color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{2,-92},{-21,-92}}, color={0,0,255}));
          connect(pwShuntC.p, motorTypeIII.p) annotation (Line(points={{42,-36},{
                  18,-36},{18,0},{42,0}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-206,-92},{-158,-92}}, color={0,0,255}));
          connect(pwLine3.p, OneBus.p) annotation (Line(points={{-155,-20},{-176,
                  -20},{-176,4},{-200,4}}, color={0,0,255}));
          connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-137,-20},{
                  -122,-20},{-122,0},{-72,0},{-72,-92}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                    -280,-140},{140,80}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
        end LF4AllInOneUnfaulted;

        model LF4AllInOneFaulted
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-168,-102},{-148,-82}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1,
            angle_0=-9.4,
            Q_0=213,
            ra=0,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-250,-114},{-206,-70}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-124,-102},{-104,-82}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=1200,
            Q_0=53)
            annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{-8,-102},{12,-82}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0024,
            angle_0=-13.3,
            P_0=500,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-22,-22},{22,22}},
                rotation=90,
                origin={66,-92})));
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.04,
            angle_0=0,
            P_0=800,
            Q_0=214.18834,
            displayPF=true)
            annotation (Placement(transformation(extent={{-274,-8},{-236,16}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-210,-6},{-190,14}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            t1=1,
            opening=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-156,20},{-136,40}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-138,-68},{-118,-48}})));
          OpenIPSL.Electrical.Buses.Bus FourBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-300,
            Q_0=-23)
            annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer2(
            Sn=750,
            V_b=380,
            Vn=380,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1)
            annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
          OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
            Q_0=0,
            Hm=0.6,
            V_b=15,
            Sup=0,
            V_0=1.04,
            angle_0=-0.3,
            Rs=0.031,
            Xs=0.1,
            Rr1=0.05,
            Xr1=0.07,
            Xm=3.20,
            a=0.78,
            P_0=0.01333)
            annotation (Placement(transformation(extent={{62,-10},{42,10}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-40,-102},{-20,-82}})));
          OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Qnom=117, Vbase=15)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={52,-36})));
          OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true,
                                                     t_o=1) annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=0,
                origin={-176,-60})));
          Components.RelayPackOCR.LatchPowerToRealRecordOverCurrrentRelay
            latchPowerToRealRecordOverCurrrentRelay annotation (Placement(
                transformation(
                extent={{-14,-10},{14,10}},
                rotation=-90,
                origin={-188,-26})));
          OpenIPSL.Electrical.Events.PwFault pwFault(
            R=0,
            t1=2,
            t2=2.14,
            X=0.3) annotation (Placement(transformation(extent={{-88,-48},{-76,
                    -36}})));
        equation
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-245.6,
                  -67.8},{-245.6,-64},{-262,-64},{-262,-81},{-254.4,-81}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-245.6,
                  -116.2},{-245.6,-120},{-262,-120},{-262,-103},{-254.4,-103}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-158,-92},{-125,-92}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p)
            annotation (Line(points={{2,-92},{44,-92}},   color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-103,-92},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.p,OneBus. p) annotation (Line(points={{-155,30},{-176,
                  30},{-176,4},{-200,4}},
                                      color={0,0,255}));
          connect(OneBus.p, infiniteBus.p)
            annotation (Line(points={{-200,4},{-236,4}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-137,30},{-122,
                  30},{-122,0},{-72,0},{-72,-92}}, color={0,0,255}));
          connect(FourBus.p, twoWindingTransformer2.n)
            annotation (Line(points={{-6,0},{-25,0}}, color={0,0,255}));
          connect(FourBus.p, motorTypeIII.p)
            annotation (Line(points={{-6,0},{42,0}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine2.p)
            annotation (Line(points={{-72,-92},{-39,-92}}, color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{2,-92},{-21,-92}}, color={0,0,255}));
          connect(pwShuntC.p, motorTypeIII.p) annotation (Line(points={{42,-36},{
                  18,-36},{18,0},{42,0}}, color={0,0,255}));
          connect(pwLine3.p, breaker.r) annotation (Line(points={{-137,-58},{
                  -166,-58},{-166,-60}},
                                    color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-206,-92},{-158,-92}}, color={0,0,255}));
          connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-119,-58},{
                  -110,-58},{-110,-10},{-122,-10},{-122,0},{-72,0},{-72,-92}},
                color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay.u, OneBus.p)
            annotation (Line(points={{-187.6,-11.3412},{-187.6,4},{-200,4}},
                color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay.y, breaker.s)
            annotation (Line(points={{-188,-40.4941},{-188,-50},{-198,-50},{
                  -198,-60},{-186,-60}}, color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay.TripSingal, breaker.Trigger)
            annotation (Line(points={{-177,-25.5059},{-176,-25.5059},{-176,-48}},
                color={255,0,255}));
          connect(pwFault.p, ThreeBus.p) annotation (Line(points={{-89,-42},{
                  -96,-42},{-96,0},{-72,0},{-72,-92}}, color={0,0,255}));
          connect(twoWindingTransformer2.p, ThreeBus.p) annotation (Line(points=
                 {{-47,0},{-62,0},{-62,-92},{-72,-92}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                    -280,-140},{140,80}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
        end LF4AllInOneFaulted;

        model LF4AllInOneRelay
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-168,-102},{-148,-82}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1,
            angle_0=-9.4,
            Q_0=213,
            ra=0,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-250,-114},{-206,-70}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-124,-102},{-104,-82}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=1200,
            Q_0=53)
            annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{-8,-102},{12,-82}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0024,
            angle_0=-13.3,
            P_0=500,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-22,-22},{22,22}},
                rotation=90,
                origin={66,-92})));
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.04,
            angle_0=0,
            P_0=800,
            Q_0=214.18834,
            displayPF=true)
            annotation (Placement(transformation(extent={{-274,-8},{-236,16}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-210,-6},{-190,14}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            t1=1,
            opening=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-156,20},{-136,40}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-130,-46},{-110,-26}})));
          OpenIPSL.Electrical.Buses.Bus FourBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-300,
            Q_0=-23)
            annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer2(
            Sn=750,
            V_b=380,
            Vn=380,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1)
            annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
          OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
            Q_0=0,
            Hm=0.6,
            V_b=15,
            Sup=0,
            V_0=1.04,
            angle_0=-0.3,
            Rs=0.031,
            Xs=0.1,
            Rr1=0.05,
            Xr1=0.07,
            Xm=3.20,
            a=0.78,
            P_0=0.01333)
            annotation (Placement(transformation(extent={{62,-10},{42,10}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-40,-102},{-20,-82}})));
          OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Qnom=117, Vbase=15)
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={52,-36})));
          OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
            annotation (Placement(transformation(extent={{-162,-46},{-142,-26}})));
          Components.RelayPackOCR.LatchPowerToRealRecordOverCurrrentRelay
            latchPowerToRealRecordOverCurrrentRelay(Is=0.15) annotation (
              Placement(transformation(
                extent={{-14,-10},{14,10}},
                rotation=-90,
                origin={-178,-16})));
        equation
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-245.6,
                  -67.8},{-245.6,-64},{-262,-64},{-262,-81},{-254.4,-81}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-245.6,
                  -116.2},{-245.6,-120},{-262,-120},{-262,-103},{-254.4,-103}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-158,-92},{-125,-92}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p)
            annotation (Line(points={{2,-92},{44,-92}},   color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-103,-92},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.p,OneBus. p) annotation (Line(points={{-155,30},{-176,
                  30},{-176,4},{-200,4}},
                                      color={0,0,255}));
          connect(OneBus.p, infiniteBus.p)
            annotation (Line(points={{-200,4},{-236,4}}, color={0,0,255}));
          connect(FourBus.p, twoWindingTransformer2.n)
            annotation (Line(points={{-6,0},{-25,0}}, color={0,0,255}));
          connect(FourBus.p, motorTypeIII.p)
            annotation (Line(points={{-6,0},{42,0}}, color={0,0,255}));
          connect(twoWindingTransformer2.p, ThreeBus.p) annotation (Line(points={
                  {-47,0},{-72,0},{-72,-92}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine2.p)
            annotation (Line(points={{-72,-92},{-39,-92}}, color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{2,-92},{-21,-92}}, color={0,0,255}));
          connect(pwShuntC.p, motorTypeIII.p) annotation (Line(points={{42,-36},{
                  18,-36},{18,0},{42,0}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-206,-92},{-158,-92}}, color={0,0,255}));
          connect(pwLine3.p, breaker.r) annotation (Line(points={{-129,-36},{
                  -142,-36}},                  color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay.u, OneBus.p)
            annotation (Line(points={{-177.6,-1.34118},{-177.6,4},{-200,4}},
                color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay.TripSingal, breaker.Trigger)
            annotation (Line(points={{-167,-15.5059},{-152,-15.5059},{-152,-24}},
                color={255,0,255}));
          connect(breaker.s, latchPowerToRealRecordOverCurrrentRelay.y)
            annotation (Line(points={{-162,-36},{-178,-36},{-178,-30.4941}},
                color={0,0,255}));
          connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-111,-36},{
                  -100,-36},{-100,0},{-72,0},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-137,30},{
                  -100,30},{-100,0},{-72,0},{-72,-92}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                    -280,-140},{140,80}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
        end LF4AllInOneRelay;
      end LF4;

      package LF1
        model LF1AllInOneUnfaulted
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
            annotation (Placement(transformation(extent={{-244,0},{-224,20}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-198,26},{-178,46}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-198,-24},{-178,-4}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=-350,
            Q_0=-16.681575,
            displayPF=true)
            annotation (Placement(transformation(extent={{-272,0},{-252,20}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-174,-102},{-154,-82}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=450,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1.04,
            angle_0=8.8,
            Q_0=99,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-238,-114},{-194,-70}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0675,
            angle_0=4.7,
            P_0=100,
            Q_0=20)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={63,-91})));
        equation
          connect(pwLine1.p,OneBus. p) annotation (Line(points={{-197,36},{-210,
                  36},{-210,10},{-234,10}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-197,-14},{
                  -210,-14},{-210,10},{-234,10}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-252,10},{-234,10}},
                                                         color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-194,-92},{-164,-92}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-233.6,
                  -67.8},{-233.6,-62},{-252,-62},{-252,-81},{-242.4,-81}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-233.6,
                  -116.2},{-233.6,-120},{-250,-120},{-250,-103},{-242.4,-103}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{40,
                  -92},{40,-91},{46,-91}}, color={0,0,255}));
          connect(pwLine2.p, ThreeBus.p)
            annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-179,36},{
                  -146,36},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-179,-14},{
                  -146,-14},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                    -280,-140},{140,80}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
        end LF1AllInOneUnfaulted;

        model LF1AllInOneFaulted
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
            annotation (Placement(transformation(extent={{-244,0},{-224,20}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-186,26},{-166,46}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-174,-48},{-154,-28}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=-350,
            Q_0=-16.681575,
            displayPF=true)
            annotation (Placement(transformation(extent={{-272,0},{-252,20}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-174,-102},{-154,-82}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=450,
            M=7,
            Sn=500,
            D=1,
            V_0=1.04,
            angle_0=8.8,
            Q_0=99,
            ra=0.001,
            x1d=0.4148,
            w(fixed=false))
            annotation (Placement(transformation(extent={{-238,-114},{-194,-70}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0675,
            angle_0=4.7,
            P_0=100,
            Q_0=20)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={63,-91})));
          OpenIPSL.Electrical.Events.PwFault pwFault(
            R=0,
            X=0.3,
            t1=1,
            t2=1.14)
                   annotation (Placement(transformation(extent={{-94,-46},{-82,
                    -34}})));
          Components.RelayPackOCR.LatchPowerToRealRecordOverCurrrentRelay
            latchPowerToRealRecordOverCurrrentRelay1(
            Is=0.73,
            k=0.00001,
            TMS=0.2149210904) annotation (Placement(transformation(extent={{-244,
                    -48},{-210,-28}})));
          OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
            annotation (Placement(transformation(extent={{-202,-48},{-182,-28}})));
        equation
          connect(pwLine1.p,OneBus. p) annotation (Line(points={{-185,36},{-210,
                  36},{-210,10},{-234,10}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-252,10},{-234,10}},
                                                         color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-194,-92},{-164,-92}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-233.6,
                  -67.8},{-233.6,-62},{-252,-62},{-252,-81},{-242.4,-81}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-233.6,
                  -116.2},{-233.6,-120},{-250,-120},{-250,-103},{-242.4,-103}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{40,
                  -92},{40,-91},{46,-91}}, color={0,0,255}));
          connect(pwLine2.p, ThreeBus.p)
            annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-167,36},{
                  -146,36},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(pwFault.p, ThreeBus.p) annotation (Line(points={{-95,-40},{
                  -122,-40},{-122,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-155,-38},{
                  -146,-38},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay1.TripSingal, breaker.Trigger)
            annotation (Line(points={{-227.6,-27},{-227.6,-16},{-192,-16},{-192,
                  -26}}, color={255,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay1.y, breaker.s)
            annotation (Line(points={{-209.4,-38},{-202,-38}}, color={0,0,255}));
          connect(breaker.r, pwLine3.p)
            annotation (Line(points={{-182,-38},{-173,-38}}, color={0,0,255}));
          connect(latchPowerToRealRecordOverCurrrentRelay1.u, OneBus.p)
            annotation (Line(points={{-244.8,-37.6},{-250,-37.6},{-250,-12},{
                  -210,-12},{-210,10},{-234,10}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                    -280,-140},{140,80}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
        end LF1AllInOneFaulted;

        model LF1AllInOneFaultedRelayNoLatch
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
            annotation (Placement(transformation(extent={{-244,0},{-224,20}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-186,26},{-166,46}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-174,-48},{-154,-28}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=-350,
            Q_0=-16.681575,
            displayPF=true)
            annotation (Placement(transformation(extent={{-272,0},{-252,20}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-174,-102},{-154,-82}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=450,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1.04,
            angle_0=8.8,
            Q_0=99,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-238,-114},{-194,-70}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0675,
            angle_0=4.7,
            P_0=100,
            Q_0=20)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={63,-91})));
          OpenIPSL.Electrical.Events.PwFault pwFault(
            R=0,
            X=0.3,
            t1=1,
            t2=1.14)
                   annotation (Placement(transformation(extent={{-92,-46},{-80,
                    -34}})));
          OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=false)
            annotation (Placement(transformation(extent={{-202,-48},{-182,-28}})));
          Components.RelayPackOCR.NoLatchPowerToRealRecordOverCurrrentRelay
            noLatchPowerToRealRecordOverCurrrentRelay(Is=0.2) annotation (
              Placement(transformation(extent={{-242,-48},{-208,-28}})));
        equation
          connect(pwLine1.p,OneBus. p) annotation (Line(points={{-185,36},{-210,
                  36},{-210,10},{-234,10}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-252,10},{-234,10}},
                                                         color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-194,-92},{-164,-92}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-233.6,
                  -67.8},{-233.6,-62},{-252,-62},{-252,-81},{-242.4,-81}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-233.6,
                  -116.2},{-233.6,-120},{-250,-120},{-250,-103},{-242.4,-103}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{40,
                  -92},{40,-91},{46,-91}}, color={0,0,255}));
          connect(pwLine2.p, ThreeBus.p)
            annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-167,36},{
                  -146,36},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(pwFault.p, ThreeBus.p) annotation (Line(points={{-93,-40},{
                  -122,-40},{-122,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-155,-38},{
                  -146,-38},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
          connect(breaker.r, pwLine3.p)
            annotation (Line(points={{-182,-38},{-173,-38}}, color={0,0,255}));
          connect(noLatchPowerToRealRecordOverCurrrentRelay.y, breaker.s)
            annotation (Line(points={{-207.4,-38},{-202,-38}}, color={0,0,255}));
          connect(noLatchPowerToRealRecordOverCurrrentRelay.u, OneBus.p)
            annotation (Line(points={{-242.8,-37.6},{-254,-37.6},{-254,-14},{
                  -222,-14},{-222,10},{-234,10}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                    -280,-140},{140,80}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
        end LF1AllInOneFaultedRelayNoLatch;
      end LF1;

      package LF2
        model LF2AllInOneUnfaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-6,46},{14,66}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{114,-54},{134,-34}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{88,-54},{108,-34}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0411,
            angle_0=-1.6,
            P_0=400,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={161,-43})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
            annotation (Placement(transformation(extent={{30,-54},{50,-34}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-36,-54},{-16,-34}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=350,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1.01,
            angle_0=1,
            Q_0=47,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-100,-66},{-56,-22}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-4,-54},{16,-34}})));
        equation
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-5,56},{-18,56},
                  {-18,30},{-42,30}}, color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-5,6},{-18,6},{
                  -18,30},{-42,30}},  color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{124,-44},{107,-44}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{124,-44},{136,
                  -44},{136,-43},{144,-43}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-56,-44},{-26,-44}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-95.6,
                  -19.8},{-95.6,-10},{-114,-10},{-114,-33},{-104.4,-33}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-95.6,
                  -68.2},{-95.6,-72},{-120,-72},{-120,-55},{-104.4,-55}},  color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-26,-44},{-5,-44}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{40,-44},{17,-44}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine1.p)
            annotation (Line(points={{40,-44},{89,-44}}, color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{13,56},{32,56},
                  {32,6},{13,6}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{40,-44},{40,
                  32},{32,32},{32,6},{13,6}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={
                    {-180,-100},{220,100}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-180,-100},{220,100}})));
        end LF2AllInOneUnfaulted;

        model LF2AllInOneFaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            t1=1,
            opening=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-6,46},{14,66}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            t1=1,
            opening=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{114,-54},{134,-34}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{88,-54},{108,-34}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0411,
            angle_0=-1.6,
            P_0=400,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={161,-43})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
            annotation (Placement(transformation(extent={{30,-54},{50,-34}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-36,-54},{-16,-34}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=350,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1.01,
            angle_0=1,
            Q_0=47,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-100,-66},{-56,-22}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-4,-54},{16,-34}})));
        equation
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-5,56},{-18,56},
                  {-18,30},{-42,30}}, color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-5,6},{-18,6},{
                  -18,30},{-42,30}},  color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{124,-44},{107,-44}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{124,-44},{136,
                  -44},{136,-43},{144,-43}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-56,-44},{-26,-44}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-95.6,
                  -19.8},{-95.6,-10},{-114,-10},{-114,-33},{-104.4,-33}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-95.6,
                  -68.2},{-95.6,-72},{-120,-72},{-120,-55},{-104.4,-55}},  color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-26,-44},{-5,-44}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{40,-44},{17,-44}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine1.p)
            annotation (Line(points={{40,-44},{89,-44}}, color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{13,56},{32,56},
                  {32,6},{13,6}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{40,-44},{40,
                  32},{32,32},{32,6},{13,6}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={
                    {-180,-100},{220,100}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-180,-100},{220,100}})));
        end LF2AllInOneFaulted;
      end LF2;

      package LF3
        model LF3AllInOneUnfaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750) annotation (Placement(
                transformation(extent={{-88,-92},{-68,-72}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-150,-92},{-130,-72}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=350,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1.01,
            angle_0=1.0,
            Q_0=50,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-214,-104},{-170,-60}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-118,-92},{-98,-72}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-208,4},{-188,24}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-162,30},{-142,50}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-162,-20},{-142,0}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-236,4},{-216,24}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{-16,-92},{4,-72}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-42,-92},{-22,-72}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=500,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={29,-81})));
        equation
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-170,-82},{-140,-82}}, color={0,0,255}));
          connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-209.6,
                  -57.8},{-209.6,-54},{-234,-54},{-234,-71},{-218.4,-71}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-209.6,
                  -106.2},{-209.6,-112},{-228,-112},{-228,-93},{-218.4,-93}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-140,-82},{-119,-82}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{-78,-82},{-97,-82}}, color={0,0,255}));
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-161,40},{-174,
                  40},{-174,14},{-198,14}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-161,-10},{
                  -174,-10},{-174,14},{-198,14}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-216,14},{-198,14}},
                                                         color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-143,40},{
                  -122,40},{-122,-10},{-143,-10}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-78,-82},{
                  -78,16},{-122,16},{-122,-10},{-143,-10}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{-6,-82},{-23,-82}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{-6,-82},{6,-82},
                  {6,-81},{12,-81}}, color={0,0,255}));
          connect(pwLine1.p, ThreeBus.p)
            annotation (Line(points={{-41,-82},{-78,-82}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF3AllInOneUnfaulted;

        model LF3AllInOneFaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750) annotation (Placement(
                transformation(extent={{-88,-92},{-68,-72}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-150,-92},{-130,-72}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=350,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1.01,
            angle_0=1.0,
            Q_0=50,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-214,-104},{-170,-60}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-118,-92},{-98,-72}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-208,4},{-188,24}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            t1=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-162,30},{-142,50}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            t1=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-162,-20},{-142,0}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.05999999,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-236,4},{-216,24}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{-16,-92},{4,-72}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-42,-92},{-22,-72}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=500,
            Q_0=80)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={29,-81})));
          OpenIPSL.Electrical.Events.PwFault pwFault(
            R=0,
            X=0.3,
            t1=2,
            t2=2.14)
                   annotation (Placement(transformation(extent={{-94,-46},{-82,
                    -34}})));
        equation
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-170,-82},{-140,-82}}, color={0,0,255}));
          connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-209.6,
                  -57.8},{-209.6,-54},{-234,-54},{-234,-71},{-218.4,-71}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-209.6,
                  -106.2},{-209.6,-112},{-228,-112},{-228,-93},{-218.4,-93}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-140,-82},{-119,-82}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{-78,-82},{-97,-82}}, color={0,0,255}));
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-161,40},{-174,
                  40},{-174,14},{-198,14}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-161,-10},{
                  -174,-10},{-174,14},{-198,14}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-216,14},{-198,14}},
                                                         color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-143,40},{
                  -122,40},{-122,-10},{-143,-10}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-78,-82},{
                  -78,16},{-122,16},{-122,-10},{-143,-10}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{-6,-82},{-23,-82}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{-6,-82},{6,-82},
                  {6,-81},{12,-81}}, color={0,0,255}));
          connect(pwLine1.p, ThreeBus.p)
            annotation (Line(points={{-41,-82},{-78,-82}}, color={0,0,255}));
          connect(pwFault.p, pwLine3.n) annotation (Line(points={{-95,-40},{
                  -102,-40},{-102,16},{-122,16},{-122,-10},{-143,-10}}, color={
                  0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF3AllInOneFaulted;
      end LF3;

      package LF5
        model LF5AllInOneUnfaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750) annotation (Placement(
                transformation(extent={{-64,-78},{-44,-58}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-130,-78},{-110,-58}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=15,
            V_0=1.01,
            angle_0=-10,
            Q_0=37,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-194,-90},{-150,-46}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-98,-78},{-78,-58}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-206,22},{-186,42}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-160,48},{-140,68}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-160,-2},{-140,18}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-234,22},{-214,42}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{6,-78},{26,-58}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-20,-78},{0,-58}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=1200,
            Q_0=0) annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={51,-67})));
        equation
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-150,-68},{-120,-68}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-189.6,
                  -43.8},{-189.6,-36},{-228,-36},{-228,-57},{-198.4,-57}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-189.6,
                  -92.2},{-189.6,-108},{-228,-108},{-228,-79},{-198.4,-79}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-120,-68},{-99,-68}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{-54,-68},{-77,-68}}, color={0,0,255}));
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-159,58},{-172,
                  58},{-172,32},{-196,32}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-159,8},{-172,
                  8},{-172,32},{-196,32}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-214,32},{-196,32}},
                                                         color={0,0,255}));
          connect(pwLine3.n, pwLine2.n) annotation (Line(points={{-141,8},{-124,
                  8},{-124,58},{-141,58}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine2.n) annotation (Line(points={{-54,-68},{
                  -54,32},{-124,32},{-124,58},{-141,58}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine1.p)
            annotation (Line(points={{-54,-68},{-19,-68}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{16,-68},{-1,-68}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{16,-68},{28,
                  -68},{28,-67},{34,-67}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF5AllInOneUnfaulted;

        model LF5AllInOneFaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750) annotation (Placement(
                transformation(extent={{-64,-78},{-44,-58}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-130,-78},{-110,-58}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=15,
            V_0=1.01,
            angle_0=-10,
            Q_0=37,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-194,-90},{-150,-46}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-98,-78},{-78,-58}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-206,22},{-186,42}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-160,48},{-140,68}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            t1=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-160,-2},{-140,18}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-234,22},{-214,42}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{6,-78},{26,-58}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-20,-78},{0,-58}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=1200,
            Q_0=0) annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={51,-67})));
        equation
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-150,-68},{-120,-68}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-189.6,
                  -43.8},{-189.6,-36},{-228,-36},{-228,-57},{-198.4,-57}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-189.6,
                  -92.2},{-189.6,-108},{-228,-108},{-228,-79},{-198.4,-79}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-120,-68},{-99,-68}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{-54,-68},{-77,-68}}, color={0,0,255}));
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-159,58},{-172,
                  58},{-172,32},{-196,32}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-159,8},{-172,
                  8},{-172,32},{-196,32}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-214,32},{-196,32}},
                                                         color={0,0,255}));
          connect(pwLine3.n, pwLine2.n) annotation (Line(points={{-141,8},{-124,
                  8},{-124,58},{-141,58}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine2.n) annotation (Line(points={{-54,-68},{
                  -54,32},{-124,32},{-124,58},{-141,58}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine1.p)
            annotation (Line(points={{-54,-68},{-19,-68}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{16,-68},{-1,-68}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{16,-68},{28,
                  -68},{28,-67},{34,-67}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF5AllInOneFaulted;
      end LF5;

      package LF6
        model LF6AllInOneUnfaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750) annotation (Placement(
                transformation(extent={{-70,-86},{-50,-66}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-136,-86},{-116,-66}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=15,
            V_0=1.01,
            angle_0=-14.8,
            Q_0=213,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-200,-98},{-156,-54}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-104,-86},{-84,-66}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-202,28},{-182,48}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-156,54},{-136,74}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-156,4},{-136,24}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-230,28},{-210,48}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{8,-86},{28,-66}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-18,-86},{2,-66}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=1500,
            Q_0=150)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={53,-75})));
        equation
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-156,-76},{-126,-76}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-195.6,
                  -51.8},{-195.6,-44},{-234,-44},{-234,-65},{-204.4,-65}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-195.6,
                  -100.2},{-195.6,-116},{-234,-116},{-234,-87},{-204.4,-87}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-126,-76},{-105,-76}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{-60,-76},{-83,-76}}, color={0,0,255}));
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-155,64},{-168,
                  64},{-168,38},{-192,38}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-155,14},{-168,
                  14},{-168,38},{-192,38}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-210,38},{-192,38}},
                                                         color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-137,64},{
                  -124,64},{-124,14},{-137,14}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-60,-76},{
                  -60,40},{-124,40},{-124,14},{-137,14}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine1.p)
            annotation (Line(points={{-60,-76},{-17,-76}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{18,-76},{1,-76}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{18,-76},{30,
                  -76},{30,-75},{36,-75}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF6AllInOneUnfaulted;

        model LF6AllInOneFaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750) annotation (Placement(
                transformation(extent={{-70,-86},{-50,-66}})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-136,-86},{-116,-66}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=15,
            V_0=1.01,
            angle_0=-14.8,
            Q_0=213,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-200,-98},{-156,-54}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-104,-86},{-84,-66}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-202,28},{-182,48}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-156,54},{-136,74}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            t1=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-156,4},{-136,24}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-230,28},{-210,48}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{8,-86},{28,-66}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-18,-86},{2,-66}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=1500,
            Q_0=150)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={53,-75})));
        equation
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-156,-76},{-126,-76}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-195.6,
                  -51.8},{-195.6,-44},{-234,-44},{-234,-65},{-204.4,-65}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-195.6,
                  -100.2},{-195.6,-116},{-234,-116},{-234,-87},{-204.4,-87}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-126,-76},{-105,-76}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n)
            annotation (Line(points={{-60,-76},{-83,-76}}, color={0,0,255}));
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-155,64},{-168,
                  64},{-168,38},{-192,38}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-155,14},{-168,
                  14},{-168,38},{-192,38}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-210,38},{-192,38}},
                                                         color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-137,64},{
                  -124,64},{-124,14},{-137,14}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-60,-76},{
                  -60,40},{-124,40},{-124,14},{-137,14}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine1.p)
            annotation (Line(points={{-60,-76},{-17,-76}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{18,-76},{1,-76}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{18,-76},{30,
                  -76},{30,-75},{36,-75}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF6AllInOneFaulted;
      end LF6;

      package LF7
        model LF7AllInOneUnfaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-236,76},{-176,96}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-198,16},{-178,36}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-450,
            Q_0=-15)
            annotation (Placement(transformation(extent={{-72,-72},{-52,-52}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-152,42},{-132,62}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-152,-8},{-132,12}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=150,
            Q_0=66,
            displayPF=true)
            annotation (Placement(transformation(extent={{-226,16},{-206,36}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{8,-72},{28,-52}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-18,-72},{2,-52}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0404,
            angle_0=-3.2,
            P_0=1500,
            Q_0=150)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={53,-61})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-138,-72},{-118,-52}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=450,
            M=7,
            Sn=500,
            w(fixed=true),
            D=15,
            V_0=1.01,
            angle_0=-11.1,
            Q_0=198,
            ra=0.001,
            x1d=0.4148)
            annotation (Placement(transformation(extent={{-202,-84},{-158,-40}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-106,-72},{-86,-52}})));
        equation
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-151,52},{-164,
                  52},{-164,26},{-188,26}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-151,2},{-164,
                  2},{-164,26},{-188,26}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-206,26},{-188,26}},
                                                         color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{18,-62},{1,-62}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{18,-62},{30,
                  -62},{30,-61},{36,-61}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-158,-62},{-128,-62}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-197.6,
                  -37.8},{-197.6,-30},{-236,-30},{-236,-51},{-206.4,-51}},
                                                                      color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-197.6,
                  -86.2},{-197.6,-102},{-236,-102},{-236,-73},{-206.4,-73}},
                                                                           color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-128,-62},{-107,-62}}, color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-85,-62},{-62,-62}}, color={0,0,255}));
          connect(pwLine1.p, ThreeBus.p)
            annotation (Line(points={{-17,-62},{-62,-62}}, color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-133,52},{
                  -112,52},{-112,2},{-133,2}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-62,-62},{
                  -62,26},{-112,26},{-112,2},{-133,2}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
                    -120},{100,100}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-240,-120},{100,100}})));
        end LF7AllInOneUnfaulted;
      end LF7;

      package LF8
        model LF8AllInOneUnfaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-96,78},{-36,98}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-68,34},{-48,54}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-450,
            Q_0=-15)
            annotation (Placement(transformation(extent={{48,-46},{68,-26}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-22,60},{-2,80}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=false)
            annotation (Placement(transformation(extent={{-22,10},{-2,30}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=800,
            Q_0=214,
            displayPF=true)
            annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{124,-46},{144,-26}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{98,-46},{118,-26}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0024,
            angle_0=-13.3,
            P_0=900,
            Q_0=50)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={169,-35})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
            annotation (Placement(transformation(extent={{-12,-52},{8,-32}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1,
            angle_0=-14.8,
            Q_0=177,
            ra=0.001,
            x1d=1.92)
            annotation (Placement(transformation(extent={{-76,-64},{-32,-20}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{8,-52},{28,-32}})));
          OpenIPSL.Electrical.Buses.Bus FourBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-300,
            Q_0=-23)
            annotation (Placement(transformation(extent={{100,30},{120,50}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer2(
            Sn=750,
            V_b=380,
            Vn=380,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1)
            annotation (Placement(transformation(extent={{70,30},{90,50}})));
          OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
            Q_0=0,
            Hm=0.6,
            V_b=15,
            Sup=0,
            V_0=0.9990,
            angle_0=-21.3,
            Rs=0.031,
            Xs=0.1,
            Rr1=0.05,
            Xr1=0.07,
            Xm=3.20,
            a=0.78,
            P_0=0.01333)
            annotation (Placement(transformation(extent={{178,30},{158,50}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ1(
            V_b=15,
            P_0=0,
            Q_0=-202) annotation (Placement(transformation(extent={{-10,-10},{10,
                    10}},
                rotation=90,
                origin={168,14})));
        equation
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-21,70},{-34,
                  70},{-34,44},{-58,44}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-21,20},{-34,
                  20},{-34,44},{-58,44}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-76,44},{-58,44}}, color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-3,70},{8,70},
                  {8,20},{-3,20}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{134,-36},{117,-36}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{134,-36},{146,
                  -36},{146,-35},{152,-35}}, color={0,0,255}));
          connect(pwLine1.p, ThreeBus.p)
            annotation (Line(points={{99,-36},{58,-36}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-32,-42},{-2,-42}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-71.6,
                  -17.8},{-71.6,-10},{-98,-10},{-98,-31},{-80.4,-31}},color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-71.6,
                  -66.2},{-71.6,-68},{-94,-68},{-94,-53},{-80.4,-53}},     color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-2,-42},{7,-42}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{58,-36},{58,
                  40},{8,40},{8,20},{-3,20}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n) annotation (Line(points=
                 {{58,-36},{44,-36},{44,-42},{29,-42}}, color={0,0,255}));
          connect(FourBus.p,twoWindingTransformer2. n)
            annotation (Line(points={{110,40},{91,40}},color={0,0,255}));
          connect(FourBus.p,motorTypeIII. p)
            annotation (Line(points={{110,40},{158,40}},
                                                       color={0,0,255}));
          connect(lOADPQ1.p, motorTypeIII.p) annotation (Line(points={{158,14},
                  {130,14},{130,40},{158,40}}, color={0,0,255}));
          connect(twoWindingTransformer2.p, pwLine3.n) annotation (Line(points=
                  {{69,40},{8,40},{8,20},{-3,20}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={
                    {-100,-100},{200,100}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-100,-100},{200,100}})));
        end LF8AllInOneUnfaulted;

        model LF8AllInOneFaulted
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-96,78},{-36,98}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-68,34},{-48,54}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-450,
            Q_0=-15)
            annotation (Placement(transformation(extent={{48,-46},{68,-26}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.4143333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-22,60},{-2,80}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            t1=1,
            displayPF=true)
            annotation (Placement(transformation(extent={{-22,10},{-2,30}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            V_0=1.08,
            angle_0=0,
            P_0=800,
            Q_0=214,
            displayPF=true)
            annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{124,-46},{144,-26}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{98,-46},{118,-26}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=1.0024,
            angle_0=-13.3,
            P_0=900,
            Q_0=50)
                   annotation (Placement(transformation(
                extent={{-17,-17},{17,17}},
                rotation=90,
                origin={169,-35})));
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
            annotation (Placement(transformation(extent={{-12,-52},{8,-32}})));
          OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
            Vn=20,
            V_b=20,
            P_0=300,
            M=7,
            Sn=500,
            w(fixed=true),
            D=1,
            V_0=1,
            angle_0=-14.8,
            Q_0=177,
            ra=0.001,
            x1d=1.92)
            annotation (Placement(transformation(extent={{-76,-64},{-32,-20}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{8,-52},{28,-32}})));
          OpenIPSL.Electrical.Buses.Bus FourBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=-300,
            Q_0=-23)
            annotation (Placement(transformation(extent={{100,30},{120,50}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer2(
            Sn=750,
            V_b=380,
            Vn=380,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1)
            annotation (Placement(transformation(extent={{70,30},{90,50}})));
          OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
            Q_0=0,
            Hm=0.6,
            V_b=15,
            Sup=0,
            V_0=0.9990,
            angle_0=-21.3,
            Rs=0.031,
            Xs=0.1,
            Rr1=0.05,
            Xr1=0.07,
            Xm=3.20,
            a=0.78,
            P_0=0.01333)
            annotation (Placement(transformation(extent={{178,30},{158,50}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ1(
            V_b=15,
            P_0=0,
            Q_0=-202) annotation (Placement(transformation(extent={{-10,-10},{10,
                    10}},
                rotation=90,
                origin={168,14})));
        equation
          connect(pwLine2.p,OneBus. p) annotation (Line(points={{-21,70},{-34,
                  70},{-34,44},{-58,44}},
                                      color={0,0,255}));
          connect(pwLine3.p,OneBus. p) annotation (Line(points={{-21,20},{-34,
                  20},{-34,44},{-58,44}},
                                      color={0,0,255}));
          connect(infiniteBus.p,OneBus. p)
            annotation (Line(points={{-76,44},{-58,44}}, color={0,0,255}));
          connect(pwLine2.n, pwLine3.n) annotation (Line(points={{-3,70},{8,70},
                  {8,20},{-3,20}}, color={0,0,255}));
          connect(FiveBus.p, pwLine1.n)
            annotation (Line(points={{134,-36},{117,-36}}, color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{134,-36},{146,
                  -36},{146,-35},{152,-35}}, color={0,0,255}));
          connect(pwLine1.p, ThreeBus.p)
            annotation (Line(points={{99,-36},{58,-36}}, color={0,0,255}));
          connect(order2_1.p, TwoBus.p)
            annotation (Line(points={{-32,-42},{-2,-42}}, color={0,0,255}));
          connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-71.6,
                  -17.8},{-71.6,-10},{-98,-10},{-98,-31},{-80.4,-31}},color={0,0,
                  127}));
          connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-71.6,
                  -66.2},{-71.6,-68},{-94,-68},{-94,-53},{-80.4,-53}},     color=
                  {0,0,127}));
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-2,-42},{7,-42}}, color={0,0,255}));
          connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{58,-36},{58,
                  40},{8,40},{8,20},{-3,20}}, color={0,0,255}));
          connect(ThreeBus.p, twoWindingTransformer1.n) annotation (Line(points=
                 {{58,-36},{44,-36},{44,-42},{29,-42}}, color={0,0,255}));
          connect(FourBus.p,twoWindingTransformer2. n)
            annotation (Line(points={{110,40},{91,40}},color={0,0,255}));
          connect(FourBus.p,motorTypeIII. p)
            annotation (Line(points={{110,40},{158,40}},
                                                       color={0,0,255}));
          connect(lOADPQ1.p, motorTypeIII.p) annotation (Line(points={{158,14},
                  {130,14},{130,40},{158,40}}, color={0,0,255}));
          connect(twoWindingTransformer2.p, pwLine3.n) annotation (Line(points=
                  {{69,40},{8,40},{8,20},{-3,20}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={
                    {-100,-100},{200,100}})), Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-100,-100},{200,100}})));
        end LF8AllInOneFaulted;
      end LF8;
    end Scenarios;

    model TestRelay
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-94,76},{-34,96}})));
      OpenIPSL.Electrical.Buses.Bus OneBus(
        angle_0=0,
        V_b=380,
        V_0=1.08,
        P_0=450,
        Q_0=118)
        annotation (Placement(transformation(extent={{-56,-8},{-36,12}})));
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=-450,
        Q_0=-15)
        annotation (Placement(transformation(extent={{208,-10},{228,10}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.4143333333,
        G=0,
        B=0,
        t1=1,
        opening=1,
        displayPF=true)
        annotation (Placement(transformation(extent={{-10,18},{10,38}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-10,-32},{10,-12}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        V_0=1.04,
        angle_0=0,
        P_0=800,
        Q_0=214,
        displayPF=true)
        annotation (Placement(transformation(extent={{-84,-8},{-64,12}})));
      OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
        V_b=380,
        P_0=600,
        Q_0=140) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={246,0})));
      OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
        annotation (Placement(transformation(extent={{84,-36},{104,-16}})));
      Components.RelayPackOCR.LatchPowerToRealRecordOverCurrrentRelay
        latchPowerToRealRecordOverCurrrentRelay(Is=0.3)
        annotation (Placement(transformation(extent={{46,-32},{74,-12}})));
    equation
      connect(pwLine2.p,OneBus. p) annotation (Line(points={{-9,28},{-22,28},{
              -22,2},{-46,2}},    color={0,0,255}));
      connect(pwLine3.p,OneBus. p) annotation (Line(points={{-9,-22},{-22,-22},
              {-22,2},{-46,2}},   color={0,0,255}));
      connect(pwLine2.n,ThreeBus. p) annotation (Line(points={{9,28},{160,28},{
              160,0},{218,0}},  color={0,0,255}));
      connect(infiniteBus.p,OneBus. p)
        annotation (Line(points={{-64,2},{-46,2}},   color={0,0,255}));
      connect(ThreeBus.p, lOADPQ.p)
        annotation (Line(points={{218,0},{236,0}}, color={0,0,255}));
      connect(breaker.s,latchPowerToRealRecordOverCurrrentRelay. y)
        annotation (Line(points={{84,-26},{80,-26},{80,-22},{74.4941,-22}},
                                                          color={0,0,255}));
      connect(latchPowerToRealRecordOverCurrrentRelay.TripSingal,breaker. Trigger)
        annotation (Line(points={{59.5059,-11},{59.5059,4},{94,4},{94,-14}},
            color={255,0,255}));
      connect(latchPowerToRealRecordOverCurrrentRelay.u, pwLine3.n) annotation (
         Line(points={{45.3412,-21.6},{26.6706,-21.6},{26.6706,-22},{9,-22}},
            color={0,0,255}));
      connect(breaker.r, ThreeBus.p) annotation (Line(points={{104,-26},{160,
              -26},{160,0},{218,0}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -100,-100},{260,100}})), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{260,100}})));
    end TestRelay;

    package AIOSModel

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
                RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData
            constrainedby
              RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData
            annotation(choicesAllMatching);
            Voltage voltage;

            replaceable record Power =
                RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData
            constrainedby
              RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData
            annotation (choicesAllMatching);
            Power power;
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PowerFlow;
        end Records;

        package PowerData

          record PPF1
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
          extends RelayCoordination.AIOS.AIOSModel.Data.Records.PowerPFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            extends RelayCoordination.AIOS.AIOSModel.Data.Records.VoltagePFData(
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
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF1,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF1)
                                                                 "Power FLow 1";
            record PF2 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF2,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF2)
                                                                 "Power FLow 2";
            record PF3 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF3,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF3)
                                                                 "Power FLow 3";

            record PF4 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF4,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF4)
                                                                 "Power FLow 4";
            record PF5 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF5,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF5)
                                                                 "Power FLow 5";

            record PF6 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF6,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF6)
                                                                 "Power FLow 6";

            record PF7 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF7,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF7)
                                                                 "Power FLow 7";

            record PF8 =
            RelayCoordination.AIOS.AIOSModel.Data.Records.PowerFlow (
            redeclare replaceable record Voltage =
            RelayCoordination.AIOS.AIOSModel.Data.VoltageData.VPF8,
            redeclare replaceable record Power =
            RelayCoordination.AIOS.AIOSModel.Data.PowerData.PPF8)
                                                                 "Power FLow 8";
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end SystemData;
        end SystemData;
      end Data;

      model AIOSNoMotorRelayCombined
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
          annotation (Placement(transformation(extent={{-138,-66},{-118,-46}})));
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
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1(
          Sn=500,
          V_b=20,
          Vn=20,
          S_b=750,
          rT=0,
          m=1/1.04,
          xT=0.087)
          annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0)
                 annotation (Placement(transformation(
              extent={{-17,-17},{17,17}},
              rotation=90,
              origin={73,-93})));
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
        OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
          annotation (Placement(transformation(extent={{-202,-26},{-182,-6}})));
        OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
          annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
        RelayCoordination.AIOS.Parts.PSSEGeneratorTGOV generatorTGOV(
          V_b=20,
          M_b=750,
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0) annotation (Placement(transformation(
                extent={{-252,-112},{-196,-76}})));
        OpenIPSL.Electrical.Events.PwFault pwFault(
          R=0,
          X=0.0375,
          t1=1,
          t2=1.13999)
                 annotation (Placement(transformation(extent={{-116,-46},{-104,-34}})));

       Real Imag;
        Modelica.Blocks.Sources.RealExpression realExpression(y=Imag)
          annotation (Placement(transformation(extent={{-204,12},{-184,32}})));
        Components.RelayPackOCR.ReferenceRelay luigiRelay
          annotation (Placement(transformation(extent={{-166,16},{-140,28}})));
      equation
        Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
        connect(TwoBus.p, twoWindingTransformer1.p)
          annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
        connect(twoWindingTransformer1.n, ThreeBus.p)
          annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
        connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{40,-92},
                {40,-93},{56,-93}},      color={0,0,255}));
        connect(pwLine2.p, ThreeBus.p)
          annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
        connect(pwLine3.p, breaker.r)
          annotation (Line(points={{-155,-16},{-182,-16}}, color={0,0,255}));
        connect(breaker1.Trigger, breaker.Trigger) annotation (Line(points={{-106,-4},
                {-106,4},{-192,4},{-192,-4}},          color={255,0,255}));
        connect(ThreeBus.p, breaker1.r) annotation (Line(points={{-72,-92},{-72,
                -16},{-96,-16}},                    color={0,0,255}));
        connect(generatorTGOV.pwPin, TwoBus.p) annotation (Line(points={{-194.88,
                -91.3},{-178.44,-91.3},{-178.44,-92},{-164,-92}},     color={0,
                0,255}));
        connect(infiniteBus.p, OneBus.p)
          annotation (Line(points={{-278,-16},{-234,-16}},
                                                         color={0,0,255}));
        connect(pwLine1.n, breaker1.r) annotation (Line(points={{-119,-56},{-92,
                -56},{-92,-16},{-96,-16}}, color={0,0,255}));
        connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
                -16},{-226,-56},{-137,-56}},
                                          color={0,0,255}));
        connect(pwFault.p, pwLine3.n) annotation (Line(points={{-117,-40},{-126,-40},{
                -126,-16},{-137,-16}}, color={0,0,255}));
        connect(breaker1.s, pwLine3.n)
          annotation (Line(points={{-116,-16},{-137,-16}}, color={0,0,255}));
        connect(breaker.s, pwLine1.p) annotation (Line(points={{-202,-16},{-226,
                -16},{-226,-56},{-137,-56}}, color={0,0,255}));
        connect(realExpression.y, luigiRelay.u)
          annotation (Line(points={{-183,22},{-167.444,22}}, color={0,0,127}));
        connect(luigiRelay.TripSingal, breaker.Trigger) annotation (Line(points={{
                -139.278,22},{-134,22},{-134,4},{-192,4},{-192,-4}},  color={
                255,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                  -140},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
      end AIOSNoMotorRelayCombined;
    end AIOSModel;

    package Trials
      model RecordLF1AllInOneUnfaulted
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
          annotation (Placement(transformation(extent={{-244,0},{-224,20}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine1(
          R=0,
          X=0.4143333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-198,26},{-178,46}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine3(
          R=0,
          X=0.414333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-198,-24},{-178,-4}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          V_0=1.05999999,
          angle_0=0,
          P_0=-350,
          Q_0=-16.681575,
          displayPF=true)
          annotation (Placement(transformation(extent={{-272,0},{-252,20}})));
        OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
              transformation(extent={{-174,-102},{-154,-82}})));
        OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
          Vn=20,
          V_b=20,
          P_0=450,
          M=7,
          Sn=500,
          w(fixed=true),
          D=1,
          V_0=1.04,
          angle_0=8.8,
          Q_0=99,
          ra=0.001,
          x1d=0.4148)
          annotation (Placement(transformation(extent={{-238,-114},{-194,-70}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1(
          Sn=500,
          V_b=20,
          Vn=20,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1/1.04)
          annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=1.0675,
          angle_0=4.7,
          P_0=100,
          Q_0=20)
                 annotation (Placement(transformation(
              extent={{-17,-17},{17,17}},
              rotation=90,
              origin={63,-91})));
        OpenIPSL.Electrical.Buses.Bus FourBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer2(
          Sn=750,
          V_b=380,
          Vn=380,
          S_b=750,
          rT=0,
          xT=0.08,
          m=999999)
          annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
        OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
          Q_0=0,
          Hm=0.6,
          V_b=15,
          Sup=0,
          V_0=1.04,
          angle_0=-0.3,
          Rs=0.031,
          Xs=0.1,
          Rr1=0.05,
          Xr1=0.07,
          Xm=3.20,
          a=0.78,
          P_0=0.01333)
          annotation (Placement(transformation(extent={{62,-10},{42,10}})));
        OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Qnom=117, Vbase=15)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={52,-36})));
      equation
        connect(pwLine1.p,OneBus. p) annotation (Line(points={{-197,36},{-210,
                36},{-210,10},{-234,10}},
                                    color={0,0,255}));
        connect(pwLine3.p,OneBus. p) annotation (Line(points={{-197,-14},{
                -210,-14},{-210,10},{-234,10}},
                                    color={0,0,255}));
        connect(infiniteBus.p,OneBus. p)
          annotation (Line(points={{-252,10},{-234,10}},
                                                       color={0,0,255}));
        connect(order2_1.p, TwoBus.p)
          annotation (Line(points={{-194,-92},{-164,-92}}, color={0,0,255}));
        connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-233.6,
                -67.8},{-233.6,-62},{-252,-62},{-252,-81},{-242.4,-81}},
                                                                    color={0,0,
                127}));
        connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-233.6,
                -116.2},{-233.6,-120},{-250,-120},{-250,-103},{-242.4,-103}},
                                                                         color=
                {0,0,127}));
        connect(TwoBus.p, twoWindingTransformer1.p)
          annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
        connect(twoWindingTransformer1.n, ThreeBus.p)
          annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
        connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{40,
                -92},{40,-91},{46,-91}}, color={0,0,255}));
        connect(pwLine2.p, ThreeBus.p)
          annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
        connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-179,36},{
                -146,36},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
        connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-179,-14},{
                -146,-14},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
        connect(FourBus.p,twoWindingTransformer2. n)
          annotation (Line(points={{-6,0},{-25,0}}, color={0,0,255}));
        connect(FourBus.p,motorTypeIII. p)
          annotation (Line(points={{-6,0},{42,0}}, color={0,0,255}));
        connect(pwShuntC.p,motorTypeIII. p) annotation (Line(points={{42,-36},{
                18,-36},{18,0},{42,0}}, color={0,0,255}));
        connect(twoWindingTransformer2.p, ThreeBus.p) annotation (Line(points=
               {{-47,0},{-60,0},{-60,6},{-72,6},{-72,-92}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -280,-140},{140,80}})), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
      end RecordLF1AllInOneUnfaulted;

      model RecordLF4AllInOneUnfaulted
        OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
              transformation(extent={{-168,-102},{-148,-82}})));
        OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
          Vn=20,
          V_b=20,
          P_0=300,
          M=7,
          Sn=500,
          w(fixed=true),
          D=1,
          V_0=1,
          angle_0=-9.4,
          Q_0=213,
          ra=0,
          x1d=0.4148)
          annotation (Placement(transformation(extent={{-250,-114},{-206,-70}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1(
          Sn=500,
          V_b=20,
          Vn=20,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1/1.04)
          annotation (Placement(transformation(extent={{-124,-102},{-104,-82}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=1200,
          Q_0=53)
          annotation (Placement(transformation(extent={{-82,-102},{-62,-82}})));
        OpenIPSL.Electrical.Buses.Bus FiveBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0)
          annotation (Placement(transformation(extent={{-8,-102},{12,-82}})));
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=1.0024,
          angle_0=-13.3,
          P_0=500,
          Q_0=80)
                 annotation (Placement(transformation(
              extent={{-22,-22},{22,22}},
              rotation=90,
              origin={66,-92})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          V_0=1.04,
          angle_0=0,
          P_0=800,
          Q_0=214.18834,
          displayPF=true)
          annotation (Placement(transformation(extent={{-274,-8},{-236,16}})));
        OpenIPSL.Electrical.Buses.Bus OneBus(
          angle_0=0,
          V_b=380,
          V_0=1.08,
          P_0=450,
          Q_0=118)
          annotation (Placement(transformation(extent={{-210,-6},{-190,14}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine1(
          R=0,
          X=0.4143333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-156,20},{-136,40}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine3(
          R=0,
          X=0.414333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-156,-30},{-136,-10}})));
        OpenIPSL.Electrical.Buses.Bus FourBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer2(
          Sn=750,
          V_b=380,
          Vn=380,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1)
          annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
        OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
          Q_0=0,
          Hm=0.6,
          V_b=15,
          Sup=0,
          V_0=1.04,
          angle_0=-0.3,
          Rs=0.031,
          Xs=0.1,
          Rr1=0.05,
          Xr1=0.07,
          Xm=3.20,
          a=0.78,
          P_0=0.01333)
          annotation (Placement(transformation(extent={{62,-10},{42,10}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine2(
          R=0,
          X=0.029999989612,
          G=0,
          B=0,
          displayPF=true)
          annotation (Placement(transformation(extent={{-40,-102},{-20,-82}})));
        OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Qnom=117, Vbase=15)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={52,-36})));
      equation
        connect(order2_1.vf0,order2_1. vf) annotation (Line(points={{-245.6,
                -67.8},{-245.6,-64},{-262,-64},{-262,-81},{-254.4,-81}},
                                                                    color={0,0,
                127}));
        connect(order2_1.pm0,order2_1. pm) annotation (Line(points={{-245.6,
                -116.2},{-245.6,-120},{-262,-120},{-262,-103},{-254.4,-103}},
                                                                         color=
                {0,0,127}));
        connect(TwoBus.p, twoWindingTransformer1.p)
          annotation (Line(points={{-158,-92},{-125,-92}}, color={0,0,255}));
        connect(FiveBus.p, lOADPQ.p)
          annotation (Line(points={{2,-92},{44,-92}},   color={0,0,255}));
        connect(twoWindingTransformer1.n, ThreeBus.p)
          annotation (Line(points={{-103,-92},{-72,-92}}, color={0,0,255}));
        connect(pwLine1.p,OneBus. p) annotation (Line(points={{-155,30},{-176,
                30},{-176,4},{-200,4}},
                                    color={0,0,255}));
        connect(OneBus.p, infiniteBus.p)
          annotation (Line(points={{-200,4},{-236,4}}, color={0,0,255}));
        connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-137,30},{-122,
                30},{-122,0},{-72,0},{-72,-92}}, color={0,0,255}));
        connect(FourBus.p, twoWindingTransformer2.n)
          annotation (Line(points={{-6,0},{-25,0}}, color={0,0,255}));
        connect(FourBus.p, motorTypeIII.p)
          annotation (Line(points={{-6,0},{42,0}}, color={0,0,255}));
        connect(twoWindingTransformer2.p, ThreeBus.p) annotation (Line(points={
                {-47,0},{-72,0},{-72,-92}}, color={0,0,255}));
        connect(ThreeBus.p, pwLine2.p)
          annotation (Line(points={{-72,-92},{-39,-92}}, color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{2,-92},{-21,-92}}, color={0,0,255}));
        connect(pwShuntC.p, motorTypeIII.p) annotation (Line(points={{42,-36},{
                18,-36},{18,0},{42,0}}, color={0,0,255}));
        connect(order2_1.p, TwoBus.p)
          annotation (Line(points={{-206,-92},{-158,-92}}, color={0,0,255}));
        connect(pwLine3.p, OneBus.p) annotation (Line(points={{-155,-20},{-176,
                -20},{-176,4},{-200,4}}, color={0,0,255}));
        connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-137,-20},{
                -122,-20},{-122,0},{-72,0},{-72,-92}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -280,-140},{140,80}})), Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
      end RecordLF4AllInOneUnfaulted;

      model AIOSRelayNoMotor
        OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
              transformation(extent={{-258,-104},{-234,-80}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=1200,
          Q_0=53)
          annotation (Placement(transformation(extent={{-116,-106},{-88,-78}})));
        OpenIPSL.Electrical.Buses.Bus FiveBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0)
          annotation (Placement(transformation(extent={{-6,-104},{20,-78}})));
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0)
                 annotation (Placement(transformation(
              extent={{-22,-22},{22,22}},
              rotation=90,
              origin={84,-90})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-474,50},{-408,76}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          displayPF=true,
          V_0=pF1_1.voltage.InfiniteBusV_0,
          angle_0=pF1_1.voltage.InfiniteBusangle_0,
          P_0=pF1_1.power.InfiniteBusP_0,
          Q_0=pF1_1.power.InfiniteBusQ_0)
          annotation (Placement(transformation(extent={{-480,20},{-442,44}})));
        OpenIPSL.Electrical.Buses.Bus OneBus(
          angle_0=0,
          V_b=380,
          V_0=1.08,
          P_0=450,
          Q_0=118)
          annotation (Placement(transformation(extent={{-428,18},{-406,40}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine3(
          R=0,
          X=0.414333333,
          G=0,
          B=0,
          displayPF=true)
          annotation (Placement(transformation(extent={{-346,-66},{-320,-40}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine2(
          R=0,
          X=0.029999989612,
          G=0,
          B=0,
          displayPF=true)
          annotation (Placement(transformation(extent={{-64,-104},{-40,-80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1 annotation (Placement(
              transformation(extent={{-396,58},{-376,78}})));
        Parts.PSSEGeneratorTGOV generatorTGOV(
          V_b=20,
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0,
          M_b=750) annotation (Placement(transformation(extent={{-386,-112},{-330,
                  -76}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer(
          Sn=500,
          V_b=20,
          Vn=20,
          rT=0,
          xT=0.08,
          S_b=750,
          m=1/1.04) annotation (Placement(transformation(extent={{-180,-104},
                  {-160,-84}})));
        OpenIPSL.Electrical.Events.Breaker breaker(t_o=1.14, enableTrigger=
              true) annotation (Placement(transformation(extent={{-294,-64},{
                  -274,-44}})));
        OpenIPSL.Electrical.Events.Breaker breaker1(t_o=1.14, enableTrigger=
              true) annotation (Placement(transformation(extent={{-376,-62},{
                  -356,-42}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine1(
          R=0,
          X=0.414333333,
          G=0,
          B=0,
          displayPF=true)
          annotation (Placement(transformation(extent={{-322,12},{-296,38}})));
        Components.RelayPackOCR.LatchPowerToRealRecordOverCurrrentRelay
          latchPowerToRealRecordOverCurrrentRelay annotation (Placement(
              transformation(extent={{-422,-62},{-388,-42}})));
      equation
        connect(ThreeBus.p, pwLine2.p)
          annotation (Line(points={{-102,-92},{-62.8,-92}},
                                                         color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{7,-91},{-10,-91},{-10,-92},{-41.2,-92}},
                                                       color={0,0,255}));
        connect(infiniteBus.p, OneBus.p)
          annotation (Line(points={{-442,32},{-438,32},{-438,29},{-417,29}},
                                                       color={0,0,255}));
        connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{7,-91},{35.5,
                -91},{35.5,-90},{62,-90}}, color={0,0,255}));
        connect(twoWindingTransformer.p, TwoBus.p) annotation (Line(points={{
                -181,-94},{-214,-94},{-214,-92},{-246,-92}}, color={0,0,255}));
        connect(twoWindingTransformer.n, ThreeBus.p) annotation (Line(points=
                {{-159,-94},{-130,-94},{-130,-92},{-102,-92}}, color={0,0,255}));
        connect(breaker1.r, pwLine3.p) annotation (Line(points={{-356,-52},{
                -358,-52},{-358,-53},{-344.7,-53}}, color={0,0,255}));
        connect(breaker.r, ThreeBus.p) annotation (Line(points={{-274,-54},{
                -250,-54},{-250,0},{-102,0},{-102,-92}}, color={0,0,255}));
        connect(pwLine3.n, breaker.s) annotation (Line(points={{-321.3,-53},{
                -318.65,-53},{-318.65,-54},{-294,-54}}, color={0,0,255}));
        connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-297.3,25},{
                -236,25},{-236,0},{-102,0},{-102,-92}}, color={0,0,255}));
        connect(OneBus.p, pwLine1.p) annotation (Line(points={{-417,29},{-362,
                29},{-362,25},{-320.7,25}}, color={0,0,255}));
        connect(breaker1.s, latchPowerToRealRecordOverCurrrentRelay.y)
          annotation (Line(points={{-376,-52},{-387.4,-52}}, color={0,0,255}));
        connect(latchPowerToRealRecordOverCurrrentRelay.u, pwLine1.p)
          annotation (Line(points={{-422.8,-51.6},{-442,-51.6},{-442,-6},{
                -394,-6},{-394,30},{-362,29},{-362,25},{-320.7,25}}, color={0,
                0,255}));
        connect(latchPowerToRealRecordOverCurrrentRelay.TripSingal, breaker1.Trigger)
          annotation (Line(points={{-405.6,-41},{-405.6,-24},{-366,-24},{-366,
                -40}}, color={255,0,255}));
        connect(breaker.Trigger, breaker1.Trigger) annotation (Line(points={{
                -284,-42},{-284,-24},{-366,-24},{-366,-40}}, color={255,0,255}));
        connect(TwoBus.p, generatorTGOV.pwPin) annotation (Line(points={{-246,
                -92},{-288,-92},{-288,-91},{-328.88,-91}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-480,
                  -140},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-480,-140},{140,80}})));
      end AIOSRelayNoMotor;
    end Trials;

    model AIOS
      OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
            transformation(extent={{-258,-104},{-234,-80}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
        twoWindingTransformer1(
        Sn=500,
        V_b=20,
        Vn=20,
        S_b=750,
        rT=0,
        xT=0.08,
        m=1/1.04)
        annotation (Placement(transformation(extent={{-176,-102},{-156,-82}})));
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=1200,
        Q_0=53)
        annotation (Placement(transformation(extent={{-116,-106},{-88,-78}})));
      OpenIPSL.Electrical.Buses.Bus FiveBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-15.2,
        P_0=1200,
        Q_0=0)
        annotation (Placement(transformation(extent={{-6,-104},{20,-78}})));
      OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
        V_b=380,
        V_0=pF1_1.voltage.PQLoadV_0,
        angle_0=pF1_1.voltage.PQLoadangle_0,
        P_0=pF1_1.power.PQLoadP_0,
        Q_0=pF1_1.power.PQLoadQ_0)
               annotation (Placement(transformation(
            extent={{-22,-22},{22,22}},
            rotation=90,
            origin={84,-90})));
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-474,50},{-408,76}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        displayPF=true,
        V_0=pF1_1.voltage.InfiniteBusV_0,
        angle_0=pF1_1.voltage.InfiniteBusangle_0,
        P_0=pF1_1.power.InfiniteBusP_0,
        Q_0=pF1_1.power.InfiniteBusQ_0)
        annotation (Placement(transformation(extent={{-476,-10},{-438,14}})));
      OpenIPSL.Electrical.Buses.Bus OneBus(
        angle_0=0,
        V_b=380,
        V_0=1.08,
        P_0=450,
        Q_0=118)
        annotation (Placement(transformation(extent={{-432,-10},{-410,12}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-322,-44},{-296,-18}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.029999989612,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-64,-104},{-40,-80}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-322,12},{-296,38}})));
      AIOSModel.Data.SystemData.SystemData.PF1 pF1_1
        annotation (Placement(transformation(extent={{-396,58},{-376,78}})));
      OpenIPSL.Electrical.Branches.PSAT.ULTC_VoltageControl
        uLTC_VoltageControl(
        Sn=750,
        S_b=750,
        Vbus1=380,
        Vbus2=380,
        Vn=405650,
        V_0=1.0675,
        m0=1,
        m_max=0.8,
        m_min=1.1)
        annotation (Placement(transformation(extent={{26,-102},{48,-80}})));
      Parts.PSSEGeneratorTGOV generatorTGOV(
        V_b=20,
        M_b=750,
        V_0=pF8_1.voltage.GeneratorV_0,
        angle_0=pF8_1.voltage.Generatorangle_0,
        P_0=pF8_1.power.GeneratorP_0,
        Q_0=pF8_1.power.GeneratorQ_0)
        annotation (Placement(transformation(extent={{-402,-112},{-346,-76}})));
    equation
      connect(TwoBus.p, twoWindingTransformer1.p)
        annotation (Line(points={{-246,-92},{-177,-92}}, color={0,0,255}));
      connect(twoWindingTransformer1.n, ThreeBus.p)
        annotation (Line(points={{-155,-92},{-102,-92}},color={0,0,255}));
      connect(ThreeBus.p, pwLine2.p)
        annotation (Line(points={{-102,-92},{-62.8,-92}},
                                                       color={0,0,255}));
      connect(FiveBus.p, pwLine2.n)
        annotation (Line(points={{7,-91},{-10,-91},{-10,-92},{-41.2,-92}},
                                                     color={0,0,255}));
      connect(infiniteBus.p, OneBus.p)
        annotation (Line(points={{-438,2},{-436,2},{-436,1},{-421,1}},
                                                     color={0,0,255}));
      connect(pwLine3.n, pwLine1.n) annotation (Line(points={{-297.3,-31},{
              -250,-31},{-250,25},{-297.3,25}}, color={0,0,255}));
      connect(pwLine1.p, pwLine3.p) annotation (Line(points={{-320.7,25},{
              -392,25},{-392,-31},{-320.7,-31}}, color={0,0,255}));
      connect(OneBus.p, pwLine3.p) annotation (Line(points={{-421,1},{-412,1},
              {-412,0},{-392,0},{-392,-31},{-320.7,-31}}, color={0,0,255}));
      connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-297.3,25},{
              -250,25},{-250,0},{-102,0},{-102,-92}}, color={0,0,255}));
      connect(FiveBus.p, uLTC_VoltageControl.p)
        annotation (Line(points={{7,-91},{24.9,-91}}, color={0,0,255}));
      connect(lOADPQ.p, uLTC_VoltageControl.n) annotation (Line(points={{62,
              -90},{56,-90},{56,-91},{49.1,-91}}, color={0,0,255}));
      connect(TwoBus.p, generatorTGOV.pwPin) annotation (Line(points={{-246,-92},
              {-296,-92},{-296,-91.3},{-344.88,-91.3}}, color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-480,
                -140},{140,80}})),      Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-480,-140},{140,80}})));
    end AIOS;

    model AIOSNoMotorGeneric
      inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
        annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
      AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
            AIOSModel.Data.PowerData.PPF2)
        annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
      AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
            AIOSModel.Data.PowerData.PPF1)
        annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
      AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
            AIOSModel.Data.PowerData.PPF3)
        annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
      AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
            AIOSModel.Data.PowerData.PPF5)
        annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
      AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
            AIOSModel.Data.PowerData.PPF6)
        annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
      AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
            AIOSModel.Data.PowerData.PPF7)
        annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
      OpenIPSL.Electrical.Buses.Bus ThreeBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=1200,
        Q_0=53)
        annotation (Placement(transformation(extent={{-82,-118},{-62,-98}})));
      OpenIPSL.Electrical.Buses.Bus OneBus(
        angle_0=0,
        V_b=380,
        V_0=1.08,
        P_0=450,
        Q_0=118)
        annotation (Placement(transformation(extent={{-244,-16},{-224,4}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0,
        X=0.4143333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-192,12},{-172,32}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-194,-40},{-174,-20}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        displayPF=true,
        V_0=pF2_1.voltage.InfiniteBusV_0,
        angle_0=pF2_1.voltage.InfiniteBusangle_0,
        P_0=pF2_1.power.InfiniteBusP_0,
        Q_0=pF2_1.power.InfiniteBusQ_0)
        annotation (Placement(transformation(extent={{-270,-16},{-250,4}})));
      OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
            transformation(extent={{-174,-118},{-154,-98}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
        twoWindingTransformer1(
        Sn=500,
        V_b=20,
        Vn=20,
        S_b=750,
        rT=0,
        m=1/1.04,
        xT=0.087)
        annotation (Placement(transformation(extent={{-142,-118},{-122,-98}})));
      OpenIPSL.Electrical.Buses.Bus FiveBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-15.2,
        P_0=1200,
        Q_0=0)
        annotation (Placement(transformation(extent={{18,-118},{38,-98}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine2(
        R=0,
        X=0.029999989612,
        G=0,
        B=0,
        displayPF=true)
        annotation (Placement(transformation(extent={{-28,-118},{-8,-98}})));
      OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
        V_b=380,
        V_0=pF2_1.voltage.PQLoadV_0,
        angle_0=pF2_1.voltage.PQLoadangle_0,
        P_0=pF2_1.power.PQLoadP_0,
        Q_0=pF2_1.power.PQLoadQ_0)
               annotation (Placement(transformation(
            extent={{-17,-17},{17,17}},
            rotation=90,
            origin={63,-107})));
      Parts.PSSEGeneratorTGOV generatorTGOV(
        V_b=20,
        M_b=750,
        V_0=pF2_1.voltage.GeneratorV_0,
        angle_0=pF2_1.voltage.Generatorangle_0,
        P_0=pF2_1.power.GeneratorP_0,
        Q_0=pF2_1.power.GeneratorQ_0)
        annotation (Placement(transformation(extent={{-230,-130},{-198,-92}})));
    equation
      connect(pwLine1.p,OneBus. p) annotation (Line(points={{-191,22},{-210,22},
              {-210,-6},{-234,-6}},
                                  color={0,0,255}));
      connect(pwLine3.p,OneBus. p) annotation (Line(points={{-193,-30},{-210,
              -30},{-210,-6},{-234,-6}},
                                  color={0,0,255}));
      connect(infiniteBus.p,OneBus. p)
        annotation (Line(points={{-250,-6},{-234,-6}},
                                                     color={0,0,255}));
      connect(TwoBus.p,twoWindingTransformer1. p)
        annotation (Line(points={{-164,-108},{-143,-108}},
                                                         color={0,0,255}));
      connect(twoWindingTransformer1.n,ThreeBus. p)
        annotation (Line(points={{-121,-108},{-72,-108}},
                                                        color={0,0,255}));
      connect(FiveBus.p,pwLine2. n)
        annotation (Line(points={{28,-108},{-9,-108}},
                                                     color={0,0,255}));
      connect(FiveBus.p,lOADPQ. p) annotation (Line(points={{28,-108},{40,-108},
              {40,-107},{46,-107}},    color={0,0,255}));
      connect(pwLine2.p,ThreeBus. p)
        annotation (Line(points={{-27,-108},{-72,-108}},
                                                       color={0,0,255}));
      connect(pwLine1.n,ThreeBus. p) annotation (Line(points={{-173,22},{-146,
              22},{-146,-10},{-72,-10},{-72,-108}}, color={0,0,255}));
      connect(pwLine3.n,ThreeBus. p) annotation (Line(points={{-175,-30},{-146,
              -30},{-146,-10},{-72,-10},{-72,-108}}, color={0,0,255}));
      connect(TwoBus.p, generatorTGOV.pwPin) annotation (Line(points={{-164,
              -108},{-178,-108},{-178,-108.15},{-197.36,-108.15}},
                                                                 color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -280,-140},{140,80}})), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
    end AIOSNoMotorGeneric;

    model AIOSMotorGeneric
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
        annotation (Placement(transformation(extent={{-244,0},{-224,20}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine1(
        R=0,
        X=0.4143333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-192,28},{-172,48}})));
      OpenIPSL.Electrical.Branches.PwLine pwLine3(
        R=0,
        X=0.414333333,
        G=0,
        B=0,
        displayPF=false)
        annotation (Placement(transformation(extent={{-194,-24},{-174,-4}})));
      OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
        V_b=380,
        displayPF=true,
        V_0=pF8_1.voltage.InfiniteBusV_0,
        angle_0=pF8_1.voltage.InfiniteBusangle_0,
        P_0=pF8_1.power.InfiniteBusP_0,
        Q_0=pF8_1.power.InfiniteBusQ_0)
        annotation (Placement(transformation(extent={{-270,0},{-250,20}})));
      OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
            transformation(extent={{-174,-102},{-154,-82}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
        twoWindingTransformer1(
        Sn=500,
        V_b=20,
        Vn=20,
        S_b=750,
        rT=0,
        m=1/1.04,
        xT=0.087)
        annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
      OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
        V_b=380,
        V_0=pF8_1.voltage.PQLoadV_0,
        angle_0=pF8_1.voltage.PQLoadangle_0,
        P_0=pF8_1.power.PQLoadP_0,
        Q_0=pF8_1.power.PQLoadQ_0)
               annotation (Placement(transformation(
            extent={{-17,-17},{17,17}},
            rotation=90,
            origin={63,-91})));
      AIOSModel.Data.SystemData.SystemData.PF4 pF4_1(redeclare record Voltage =
            AIOSModel.Data.VoltageData.VPF4, redeclare record Power =
            AIOSModel.Data.PowerData.PPF4)
        annotation (Placement(transformation(extent={{-196,58},{-176,78}})));
      OpenIPSL.Electrical.Buses.Bus FourBus(
        V_b=380,
        V_0=1.0455,
        angle_0=-12.7,
        P_0=-300,
        Q_0=-23)
        annotation (Placement(transformation(extent={{-16,-4},{4,16}})));
      OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
        twoWindingTransformer2(
        Sn=750,
        V_b=380,
        Vn=380,
        S_b=750,
        rT=0,
        xT=0.08,
        m=1)
        annotation (Placement(transformation(extent={{-46,-4},{-26,16}})));
      OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
        Hm=0.6,
        V_b=15,
        Sup=0,
        Rs=0.031,
        Xs=0.1,
        Rr1=0.05,
        Xr1=0.07,
        Xm=3.20,
        a=0.78,
        V_0=0.9930,
        angle_0=-15.9,
        P_0=pF8_1.power.MotorP_0,
        Q_0=pF8_1.power.MotorQ_0)
        annotation (Placement(transformation(extent={{62,-4},{42,16}})));
      OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Vbase=15, Qnom=pF8_1.power.ShuntCapacitorQnom)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={52,-30})));
      AIOSModel.Data.SystemData.SystemData.PF8 pF8_1
        annotation (Placement(transformation(extent={{-160,58},{-140,78}})));
      Parts.PSSEGeneratorTGOV generatorTGOV(
        V_b=20,
        M_b=750,
        V_0=pF8_1.voltage.GeneratorV_0,
        angle_0=pF8_1.voltage.Generatorangle_0,
        P_0=pF8_1.power.GeneratorP_0,
        Q_0=pF8_1.power.GeneratorQ_0)
        annotation (Placement(transformation(extent={{-230,-114},{-200,-74}})));
    equation
      connect(pwLine1.p,OneBus. p) annotation (Line(points={{-191,38},{-210,
              38},{-210,10},{-234,10}},
                                  color={0,0,255}));
      connect(pwLine3.p,OneBus. p) annotation (Line(points={{-193,-14},{-210,
              -14},{-210,10},{-234,10}},
                                  color={0,0,255}));
      connect(infiniteBus.p,OneBus. p)
        annotation (Line(points={{-250,10},{-234,10}},
                                                     color={0,0,255}));
      connect(TwoBus.p, twoWindingTransformer1.p)
        annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
      connect(twoWindingTransformer1.n, ThreeBus.p)
        annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
      connect(FiveBus.p, pwLine2.n)
        annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
      connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{40,
              -92},{40,-91},{46,-91}}, color={0,0,255}));
      connect(pwLine2.p, ThreeBus.p)
        annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
      connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-173,38},{-146,
              38},{-146,6},{-72,6},{-72,-92}},      color={0,0,255}));
      connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{-175,-14},{
              -146,-14},{-146,6},{-72,6},{-72,-92}}, color={0,0,255}));
      connect(FourBus.p,twoWindingTransformer2. n)
        annotation (Line(points={{-6,6},{-25,6}}, color={0,0,255}));
      connect(FourBus.p,motorTypeIII. p)
        annotation (Line(points={{-6,6},{42,6}}, color={0,0,255}));
      connect(twoWindingTransformer2.p, ThreeBus.p) annotation (Line(points={{-47,6},
              {-72,6},{-72,-92}},         color={0,0,255}));
      connect(pwShuntC.p,motorTypeIII. p) annotation (Line(points={{42,-30},{
              18,-30},{18,6},{42,6}}, color={0,0,255}));
      connect(TwoBus.p, generatorTGOV.pwPin) annotation (Line(points={{-164,-92},
              {-182,-92},{-182,-91},{-199.4,-91}},       color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                -280,-140},{140,80}})), Diagram(coordinateSystem(
              preserveAspectRatio=false, extent={{-280,-140},{140,80}})));
    end AIOSMotorGeneric;

    package PSSEAIOS
      package Parts
        model Load
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            displayPF=true,
            V_b=380,
            V_0=1.0683,
            angle_0=4.9,
            P_0=0,
            Q_0=0)
            annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
          OpenIPSL.Electrical.Buses.Bus bus1
            annotation (Placement(transformation(extent={{36,-10},{56,10}})));
          OpenIPSL.Electrical.Loads.PSSE.Load load(
            V_b=380,
            V_0=1.0683,
            angle_0=4.6,
            P_0=100,
            Q_0=20)
            annotation (Placement(transformation(extent={{80,-44},{100,-24}})));
        equation
          connect(bus1.p, load.p) annotation (Line(points={{46,0},{90,0},{90,
                  -24}}, color={0,0,255}));
          connect(bus1.p, infiniteBus.p)
            annotation (Line(points={{46,0},{0,0}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end Load;

        model PartHolding
          parameter Modelica.SIunits.PerUnit R=20 "Resistance (pu)";
          OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer
            twoWindingTransformer(
            R=0,
            X=0.08,
            G=0,
            B=0,
            VB1=20,
            VB2=380,
            ANG1=0)
            annotation (Placement(transformation(extent={{-6,-4},{6,4}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end PartHolding;

        model Transformer
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            displayPF=true,
            V_b=20,
            V_0=1.04,
            angle_0=8.8,
            P_0=450,
            Q_0=99)
            annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
          OpenIPSL.Electrical.Buses.Bus bus1
            annotation (Placement(transformation(extent={{16,-10},{36,10}})));
          OpenIPSL.Electrical.Loads.PSSE.Load load(
            V_b=380,
            V_0=1.0683,
            angle_0=4.6,
            P_0=100,
            Q_0=20)
            annotation (Placement(transformation(extent={{80,-44},{100,-24}})));
          OpenIPSL.Electrical.Buses.Bus bus2
            annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
          OpenIPSL.Electrical.Branches.PSSE.TwoWindingTransformer
            twoWindingTransformer(
            G=0,
            B=0,
            VB1=20,
            VB2=380,
            CW=1,
            CZ=1,
            R=0,
            X=0.1044534884,
            ANG1=3.075,
            t2=1.02725585)
            annotation (Placement(transformation(extent={{-20,-4},{-8,4}})));
          OpenIPSL.Electrical.Buses.Bus bus3
            annotation (Placement(transformation(extent={{58,-10},{78,10}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{42,-6},{54,6}})));
        equation
          connect(bus1.p, twoWindingTransformer.n)
            annotation (Line(points={{26,0},{-7,0}}, color={0,0,255}));
          connect(bus2.p, twoWindingTransformer.p)
            annotation (Line(points={{-50,0},{-21,0}}, color={0,0,255}));
          connect(infiniteBus.p, bus2.p)
            annotation (Line(points={{-78,0},{-50,0}}, color={0,0,255}));
          connect(bus3.p, load.p) annotation (Line(points={{68,0},{90,0},{90,
                  -24}}, color={0,0,255}));
          connect(bus1.p, pwLine2.p)
            annotation (Line(points={{26,0},{42.6,0}}, color={0,0,255}));
          connect(bus3.p, pwLine2.n)
            annotation (Line(points={{68,0},{53.4,0}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end Transformer;
      end Parts;

      model AIOSNoMotorRelayPSSE
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
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
        OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
          annotation (Placement(transformation(extent={{-202,-26},{-182,-6}})));
        OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
          annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
        .RelayCoordination.AIOS.Parts.PSSEGeneratorTGOV generatorTGOV(
          V_b=20,
          M_b=750,
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0) annotation (Placement(transformation(
                extent={{-214,-106},{-196,-82}})));
        OpenIPSL.Electrical.Events.PwFault pwFault(
          R=0,
          X=0.0375,
          t1=1,
          t2=1.13999)
                 annotation (Placement(transformation(extent={{-116,-46},{-104,-34}})));

       Real Imag;
        Modelica.Blocks.Sources.RealExpression realExpression(y=Imag)
          annotation (Placement(transformation(extent={{-202,12},{-182,32}})));
        Components.RelayPackOCR.ReferenceRelay ReferenceRelay
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
      end AIOSNoMotorRelayPSSE;

      model AIOSNoMotorPSSE
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-274,56},{-214,76}})));
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=1200,
          Q_0=53)
          annotation (Placement(transformation(extent={{-82,-122},{-54,-94}})));
        OpenIPSL.Electrical.Buses.Bus OneBus(
          angle_0=0,
          V_b=380,
          V_0=1.08,
          P_0=450,
          Q_0=118)
          annotation (Placement(transformation(extent={{-244,-16},{-224,4}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine1(
          R=0,
          X=0.4143333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-192,12},{-172,32}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine3(
          R=0,
          X=0.414333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-194,-40},{-174,-20}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          displayPF=true,
          V_0=pF1_1.voltage.InfiniteBusV_0,
          angle_0=pF1_1.voltage.InfiniteBusangle_0,
          P_0=pF1_1.power.InfiniteBusP_0,
          Q_0=pF1_1.power.InfiniteBusQ_0)
          annotation (Placement(transformation(extent={{-270,-16},{-250,4}})));
        OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
              transformation(extent={{-174,-118},{-154,-98}})));
        OpenIPSL.Electrical.Buses.Bus FiveBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0)
          annotation (Placement(transformation(extent={{18,-118},{38,-98}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine2(
          R=0,
          X=0.029999989612,
          G=0,
          B=0,
          displayPF=true)
          annotation (Placement(transformation(extent={{-28,-118},{-8,-98}})));
        RelayCoordination.AIOS.Parts.PSSEGeneratorTGOV generatorTGOV(
          V_b=20,
          M_b=750,
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0) annotation (Placement(transformation(
                extent={{-238,-138},{-198,-86}})));
        OpenIPSL.Electrical.Loads.PSSE.Load load(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0)
          annotation (Placement(transformation(extent={{50,-180},{102,-136}})));
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
          annotation (Placement(transformation(extent={{-126,-112},{-114,-104}})));
          Real Imag;
      equation
          Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
        connect(pwLine1.p,OneBus. p) annotation (Line(points={{-191,22},{-210,
                22},{-210,-6},{-234,-6}},
                                    color={0,0,255}));
        connect(pwLine3.p,OneBus. p) annotation (Line(points={{-193,-30},{-210,
                -30},{-210,-6},{-234,-6}},
                                    color={0,0,255}));
        connect(infiniteBus.p,OneBus. p)
          annotation (Line(points={{-250,-6},{-234,-6}},
                                                       color={0,0,255}));
        connect(FiveBus.p,pwLine2. n)
          annotation (Line(points={{28,-108},{-9,-108}},
                                                       color={0,0,255}));
        connect(pwLine2.p,ThreeBus. p)
          annotation (Line(points={{-27,-108},{-68,-108}},
                                                         color={0,0,255}));
        connect(pwLine1.n,ThreeBus. p) annotation (Line(points={{-173,22},{-146,
                22},{-146,-10},{-68,-10},{-68,-108}}, color={0,0,255}));
        connect(pwLine3.n,ThreeBus. p) annotation (Line(points={{-175,-30},{
                -146,-30},{-146,-10},{-68,-10},{-68,-108}},
                                                       color={0,0,255}));
        connect(TwoBus.p, generatorTGOV.pwPin) annotation (Line(points={{-164,
                -108},{-178,-108},{-178,-108.1},{-197.2,-108.1}},  color={0,0,
                255}));
        connect(FiveBus.p, load.p) annotation (Line(points={{28,-108},{76,-108},
                {76,-136}}, color={0,0,255}));
        connect(TwoBus.p, twoWindingTransformer.p)
          annotation (Line(points={{-164,-108},{-127,-108}}, color={0,0,255}));
        connect(ThreeBus.p, twoWindingTransformer.n)
          annotation (Line(points={{-68,-108},{-113,-108}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-280,
                  -200},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-280,-200},{140,80}})));
      end AIOSNoMotorPSSE;

      model AIOSNoMotorLoadVarRelayPSSE
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
          annotation (Placement(transformation(extent={{-138,-66},{-118,-46}})));
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
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
        OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
          annotation (Placement(transformation(extent={{-202,-26},{-182,-6}})));
        OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
          annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
        .RelayCoordination.AIOS.Parts.PSSEGeneratorTGOV generatorTGOV(
          V_b=20,
          M_b=750,
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0) annotation (Placement(transformation(
                extent={{-206,-106},{-188,-82}})));
        OpenIPSL.Electrical.Events.PwFault pwFault(
          R=0,
          X=0.0375,
          t1=1,
          t2=1.13999)
                 annotation (Placement(transformation(extent={{-116,-46},{-104,-34}})));

       Real Imag;
        Modelica.Blocks.Sources.RealExpression realExpression(y=Imag)
          annotation (Placement(transformation(extent={{-204,12},{-184,32}})));
        Components.RelayPackOCR.ReferenceRelay ReferenceRelay
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
        OpenIPSL.Electrical.Loads.PSSE.Load_ExtInput load_ExtInput(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0,
          d_P=1)
          annotation (Placement(transformation(extent={{76,-124},{96,-104}})));
        Modelica_Synchronous.RealSignals.Sampler.SampleClocked
                                                        sample
          annotation (Placement(transformation(extent={{-4,-150},{8,-138}})));
        Modelica_Synchronous.ClockSignals.Clocks.PeriodicExactClock periodicClock(
            resolution=Modelica_Synchronous.Types.Resolution.ms, factor=20)
          annotation (Placement(transformation(extent={{-26,-196},{-14,-184}})));
      Modelica_Synchronous.RealSignals.Sampler.Utilities.Internal.UniformNoise
        uniformNoise
        annotation (Placement(transformation(extent={{16,-154},{36,-134}})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-38,-154},{-18,-134}})));
      Modelica_Synchronous.RealSignals.Sampler.Hold hold(y_start=-1.0)
        annotation (Placement(transformation(extent={{50,-136},{62,-124}})));
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
        connect(generatorTGOV.pwPin, TwoBus.p) annotation (Line(points={{-187.64,
                -92.2},{-178.44,-92.2},{-178.44,-92},{-164,-92}},     color={0,
                0,255}));
        connect(infiniteBus.p, OneBus.p)
          annotation (Line(points={{-278,-16},{-234,-16}},
                                                         color={0,0,255}));
        connect(pwLine1.n, breaker1.r) annotation (Line(points={{-119,-56},{-92,
                -56},{-92,-16},{-96,-16}}, color={0,0,255}));
        connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
                -16},{-226,-56},{-137,-56}},
                                          color={0,0,255}));
        connect(pwFault.p, pwLine3.n) annotation (Line(points={{-117,-40},{-126,-40},{
                -126,-16},{-137,-16}}, color={0,0,255}));
        connect(breaker1.s, pwLine3.n)
          annotation (Line(points={{-116,-16},{-137,-16}}, color={0,0,255}));
        connect(breaker.s, pwLine1.p) annotation (Line(points={{-202,-16},{-226,
                -16},{-226,-56},{-137,-56}}, color={0,0,255}));
        connect(realExpression.y, ReferenceRelay.u)
          annotation (Line(points={{-183,22},{-167.444,22}}, color={0,0,127}));
        connect(ReferenceRelay.TripSingal, breaker.Trigger) annotation (Line(
              points={{-139.278,22},{-134,22},{-134,4},{-192,4},{-192,-4}},
              color={255,0,255}));
        connect(TwoBus.p, twoWindingTransformer.p)
          annotation (Line(points={{-164,-92},{-119,-92}}, color={0,0,255}));
        connect(ThreeBus.p, twoWindingTransformer.n)
          annotation (Line(points={{-72,-92},{-105,-92}}, color={0,0,255}));
        connect(load_ExtInput.p, FiveBus.p) annotation (Line(points={{86,-104},
                {86,-92},{28,-92}}, color={0,0,255}));
      connect(periodicClock.y,sample. clock) annotation (Line(
          points={{-13.4,-190},{2,-190},{2,-151.2}},
          color={175,175,175},
          pattern=LinePattern.Dot,
          thickness=0.5));
      connect(sample.y,uniformNoise. u) annotation (Line(
          points={{8.6,-144},{14,-144}},
          color={0,0,127}));
        connect(const.y, sample.u)
          annotation (Line(points={{-17,-144},{-5.2,-144}}, color={0,0,127}));
        connect(uniformNoise.y, hold.u) annotation (Line(points={{37,-144},{42,
                -144},{42,-130},{48.8,-130}}, color={0,0,127}));
        connect(load_ExtInput.u, hold.y) annotation (Line(points={{77.9,-108.5},
                {70,-108.5},{70,-130},{62.6,-130}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                  -140},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
      end AIOSNoMotorLoadVarRelayPSSE;

      model AIOSNoMotorLoadVarPSSE
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
          annotation (Placement(transformation(extent={{-180,-64},{-160,-44}})));
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
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage
            = AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
        .RelayCoordination.AIOS.Parts.PSSEGeneratorTGOV generatorTGOV(
          V_b=20,
          M_b=750,
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0) annotation (Placement(transformation(
                extent={{-206,-106},{-188,-82}})));

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
        OpenIPSL.Electrical.Loads.PSSE.Load_ExtInput load_ExtInput(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0,
          d_P=1,
          t1=1,
          d_t=5)
          annotation (Placement(transformation(extent={{76,-124},{96,-104}})));
        Modelica_Synchronous.RealSignals.Sampler.SampleClocked
                                                        sample
          annotation (Placement(transformation(extent={{-4,-150},{8,-138}})));
        Modelica_Synchronous.ClockSignals.Clocks.PeriodicExactClock periodicClock(
            resolution=Modelica_Synchronous.Types.Resolution.ms, factor=20)
          annotation (Placement(transformation(extent={{-48,-174},{-36,-162}})));
      Modelica_Synchronous.RealSignals.Sampler.Utilities.Internal.UniformNoise
        uniformNoise
        annotation (Placement(transformation(extent={{16,-154},{36,-134}})));
      Modelica.Blocks.Sources.Constant const(k=0)
        annotation (Placement(transformation(extent={{-52,-154},{-32,-134}})));
      Modelica_Synchronous.RealSignals.Sampler.Hold hold(y_start=-1.0)
        annotation (Placement(transformation(extent={{50,-136},{62,-124}})));
      equation
        Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
        connect(pwLine2.p, ThreeBus.p)
          annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
        connect(generatorTGOV.pwPin, TwoBus.p) annotation (Line(points={{-187.64,
                -92.2},{-178.44,-92.2},{-178.44,-92},{-164,-92}},     color={0,
                0,255}));
        connect(infiniteBus.p, OneBus.p)
          annotation (Line(points={{-278,-16},{-234,-16}},
                                                         color={0,0,255}));
        connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
                -16},{-226,-54},{-179,-54}},
                                          color={0,0,255}));
        connect(TwoBus.p, twoWindingTransformer.p)
          annotation (Line(points={{-164,-92},{-119,-92}}, color={0,0,255}));
        connect(ThreeBus.p, twoWindingTransformer.n)
          annotation (Line(points={{-72,-92},{-105,-92}}, color={0,0,255}));
        connect(load_ExtInput.p, FiveBus.p) annotation (Line(points={{86,-104},
                {86,-92},{28,-92}}, color={0,0,255}));
      connect(periodicClock.y,sample. clock) annotation (Line(
          points={{-35.4,-168},{2,-168},{2,-151.2}},
          color={175,175,175},
          pattern=LinePattern.Dot,
          thickness=0.5));
      connect(sample.y,uniformNoise. u) annotation (Line(
          points={{8.6,-144},{14,-144}},
          color={0,0,127}));
        connect(const.y, sample.u)
          annotation (Line(points={{-31,-144},{-5.2,-144}}, color={0,0,127}));
        connect(uniformNoise.y, hold.u) annotation (Line(points={{37,-144},{42,
                -144},{42,-130},{48.8,-130}}, color={0,0,127}));
        connect(load_ExtInput.u, hold.y) annotation (Line(points={{77.9,-108.5},
                {70,-108.5},{70,-130},{62.6,-130}}, color={0,0,127}));
        connect(pwLine3.p, pwLine1.p) annotation (Line(points={{-175,-16},{-226,
                -16},{-226,-54},{-179,-54}}, color={0,0,255}));
        connect(pwLine1.n, pwLine3.n) annotation (Line(points={{-161,-54},{-110,
                -54},{-110,-16},{-157,-16}}, color={0,0,255}));
        connect(ThreeBus.p, pwLine3.n) annotation (Line(points={{-72,-92},{-74,
                -92},{-74,-34},{-110,-34},{-110,-16},{-157,-16}}, color={0,0,
                255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                  -140},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
      end AIOSNoMotorLoadVarPSSE;
    end PSSEAIOS;

    package PSATAIOS
      model AIOSNoMotorPSAT
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
          annotation (Placement(transformation(extent={{-158,-66},{-138,-46}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine3(
          R=0,
          X=0.414333333,
          G=0,
          B=0,
          displayPF=false)
          annotation (Placement(transformation(extent={{-158,-26},{-138,-6}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          displayPF=true,
          V_0=pF1_1.voltage.InfiniteBusV_0,
          angle_0=pF1_1.voltage.InfiniteBusangle_0,
          P_0=pF1_1.power.InfiniteBusP_0,
          Q_0=pF1_1.power.InfiniteBusQ_0)
          annotation (Placement(transformation(extent={{-290,-26},{-270,-6}})));
        OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
              transformation(extent={{-174,-102},{-154,-82}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1(
          Sn=500,
          V_b=20,
          Vn=20,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1/1.04)
          annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0)
                 annotation (Placement(transformation(
              extent={{-18,-18},{18,18}},
              rotation=90,
              origin={74,-92})));
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));

       Real Imag;
        Parts.PSATGeneratorTGOV pSATGeneratorTGOV(
          V_0=pF1_1.voltage.GeneratorV_0,
          angle_0=pF1_1.voltage.Generatorangle_0,
          P_0=pF1_1.power.GeneratorP_0,
          Q_0=pF1_1.power.GeneratorQ_0) annotation (Placement(transformation(
                extent={{-246,-106},{-226,-82}})));
      equation
        Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
        connect(TwoBus.p, twoWindingTransformer1.p)
          annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
        connect(twoWindingTransformer1.n, ThreeBus.p)
          annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
        connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{56,-92}},
                                         color={0,0,255}));
        connect(pwLine2.p, ThreeBus.p)
          annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
        connect(infiniteBus.p, OneBus.p)
          annotation (Line(points={{-270,-16},{-234,-16}},
                                                         color={0,0,255}));
        connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
                -16},{-226,-56},{-157,-56}},
                                          color={0,0,255}));
        connect(pwLine3.p, pwLine1.p) annotation (Line(points={{-157,-16},{-226,
                -16},{-226,-56},{-157,-56}}, color={0,0,255}));
        connect(pwLine3.n, pwLine1.n) annotation (Line(points={{-139,-16},{-116,
                -16},{-116,-56},{-139,-56}}, color={0,0,255}));
        connect(ThreeBus.p, pwLine1.n) annotation (Line(points={{-72,-92},{-72,
                -38},{-116,-38},{-116,-56},{-139,-56}}, color={0,0,255}));
        connect(TwoBus.p, pSATGeneratorTGOV.pwPin) annotation (Line(points={{-164,
                -92},{-186,-92},{-186,-92.2},{-225.6,-92.2}},      color={0,0,
                255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                  -140},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
      end AIOSNoMotorPSAT;

      model AIOSNoMotorRelayPSAT
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
          annotation (Placement(transformation(extent={{-138,-66},{-118,-46}})));
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
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1(
          Sn=500,
          V_b=20,
          Vn=20,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1/1.04)
          annotation (Placement(transformation(extent={{-142,-102},{-122,-82}})));
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
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=pF1_1.voltage.PQLoadV_0,
          angle_0=pF1_1.voltage.PQLoadangle_0,
          P_0=pF1_1.power.PQLoadP_0,
          Q_0=pF1_1.power.PQLoadQ_0)
                 annotation (Placement(transformation(
              extent={{-18,-18},{18,18}},
              rotation=90,
              origin={74,-92})));
        AIOSModel.Data.SystemData.SystemData.PF2 pF2_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF2, redeclare record Power =
              AIOSModel.Data.PowerData.PPF2)
          annotation (Placement(transformation(extent={{-158,60},{-138,80}})));
        AIOSModel.Data.SystemData.SystemData.PF1 pF1_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF1, redeclare record Power =
              AIOSModel.Data.PowerData.PPF1)
          annotation (Placement(transformation(extent={{-188,60},{-168,80}})));
        AIOSModel.Data.SystemData.SystemData.PF3 pF3_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF3, redeclare record Power =
              AIOSModel.Data.PowerData.PPF3)
          annotation (Placement(transformation(extent={{-132,60},{-112,80}})));
        AIOSModel.Data.SystemData.SystemData.PF5 pF5_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF5, redeclare record Power =
              AIOSModel.Data.PowerData.PPF5)
          annotation (Placement(transformation(extent={{-104,60},{-84,80}})));
        AIOSModel.Data.SystemData.SystemData.PF6 pF6_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF6, redeclare record Power =
              AIOSModel.Data.PowerData.PPF6)
          annotation (Placement(transformation(extent={{-78,60},{-58,80}})));
        AIOSModel.Data.SystemData.SystemData.PF7 pF7_1(redeclare record Voltage =
              AIOSModel.Data.VoltageData.VPF7, redeclare record Power =
              AIOSModel.Data.PowerData.PPF7)
          annotation (Placement(transformation(extent={{-54,60},{-34,80}})));
        OpenIPSL.Electrical.Events.Breaker breaker(enableTrigger=true)
          annotation (Placement(transformation(extent={{-202,-26},{-182,-6}})));
        OpenIPSL.Electrical.Events.Breaker breaker1(enableTrigger=true)
          annotation (Placement(transformation(extent={{-116,-26},{-96,-6}})));
        OpenIPSL.Electrical.Events.PwFault pwFault(
          R=0,
          X=0.0375,
          t1=1,
          t2=1.13999)
                 annotation (Placement(transformation(extent={{-116,-46},{-104,-34}})));

       Real Imag;
        Modelica.Blocks.Sources.RealExpression realExpression(y=Imag)
          annotation (Placement(transformation(extent={{-204,12},{-184,32}})));
        Components.RelayPackOCR.ReferenceRelay luigiRelay
          annotation (Placement(transformation(extent={{-166,16},{-140,28}})));
      equation
        Imag =  sqrt(pwLine3.p.ir^2+pwLine3.p.ii^2);
        connect(TwoBus.p, twoWindingTransformer1.p)
          annotation (Line(points={{-164,-92},{-143,-92}}, color={0,0,255}));
        connect(twoWindingTransformer1.n, ThreeBus.p)
          annotation (Line(points={{-121,-92},{-72,-92}}, color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{28,-92},{-9,-92}}, color={0,0,255}));
        connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{28,-92},{56,-92}},
                                         color={0,0,255}));
        connect(pwLine2.p, ThreeBus.p)
          annotation (Line(points={{-27,-92},{-72,-92}}, color={0,0,255}));
        connect(pwLine3.p, breaker.r)
          annotation (Line(points={{-155,-16},{-182,-16}}, color={0,0,255}));
        connect(breaker1.Trigger, breaker.Trigger) annotation (Line(points={{-106,-4},
                {-106,4},{-192,4},{-192,-4}},          color={255,0,255}));
        connect(ThreeBus.p, breaker1.r) annotation (Line(points={{-72,-92},{-72,
                -16},{-96,-16}},                    color={0,0,255}));
        connect(infiniteBus.p, OneBus.p)
          annotation (Line(points={{-278,-16},{-234,-16}},
                                                         color={0,0,255}));
        connect(pwLine1.n, breaker1.r) annotation (Line(points={{-119,-56},{-92,
                -56},{-92,-16},{-96,-16}}, color={0,0,255}));
        connect(OneBus.p, pwLine1.p) annotation (Line(points={{-234,-16},{-226,
                -16},{-226,-56},{-137,-56}},
                                          color={0,0,255}));
        connect(pwFault.p, pwLine3.n) annotation (Line(points={{-117,-40},{-126,-40},{
                -126,-16},{-137,-16}}, color={0,0,255}));
        connect(breaker1.s, pwLine3.n)
          annotation (Line(points={{-116,-16},{-137,-16}}, color={0,0,255}));
        connect(breaker.s, pwLine1.p) annotation (Line(points={{-202,-16},{-226,
                -16},{-226,-56},{-137,-56}}, color={0,0,255}));
        connect(realExpression.y, luigiRelay.u)
          annotation (Line(points={{-183,22},{-167.444,22}}, color={0,0,127}));
        connect(luigiRelay.TripSingal, breaker.Trigger) annotation (Line(points={{
                -139.278,22},{-134,22},{-134,4},{-192,4},{-192,-4}},  color={
                255,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-300,
                  -140},{140,80}})),      Diagram(coordinateSystem(
                preserveAspectRatio=false, extent={{-300,-140},{140,80}})));
      end AIOSNoMotorRelayPSAT;
    end PSATAIOS;

    package Parts
      model PartMotor
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          V_0=1.0058,
          angle_0=-12.2,
          P_0=600,
          Q_0=140,
          displayPF=true)
          annotation (Placement(transformation(extent={{-68,14},{-48,34}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
        OpenIPSL.Electrical.Buses.Bus FourBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{20,14},{40,34}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1(
          Sn=750,
          V_b=380,
          Vn=380,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1)
          annotation (Placement(transformation(extent={{-10,14},{10,34}})));
        OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
          Q_0=0,
          Hm=0.6,
          V_b=15,
          Sup=0,
          V_0=1.04,
          angle_0=-0.3,
          Rs=0.031,
          Xs=0.1,
          Rr1=0.05,
          Xr1=0.07,
          Xm=3.20,
          a=0.78,
          P_0=0.01333)
          annotation (Placement(transformation(extent={{98,14},{78,34}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{-44,14},{-24,34}})));
        OpenIPSL.Electrical.Banks.PwShuntC pwShuntC(Qnom=117, Vbase=15)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={86,-10})));
      equation
        connect(FourBus.p, twoWindingTransformer1.n)
          annotation (Line(points={{30,24},{11,24}}, color={0,0,255}));
        connect(FourBus.p, motorTypeIII.p)
          annotation (Line(points={{30,24},{78,24}}, color={0,0,255}));
        connect(twoWindingTransformer1.p, ThreeBus.p)
          annotation (Line(points={{-11,24},{-34,24}}, color={0,0,255}));
        connect(ThreeBus.p, infiniteBus.p)
          annotation (Line(points={{-34,24},{-48,24}}, color={0,0,255}));
        connect(FourBus.p, pwShuntC.p) annotation (Line(points={{30,24},{54,24},
                {54,-10},{76,-10}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartMotor;

      model PartLoadDynamic
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
          annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=1200,
          Q_0=53)
          annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=1200,
          Q_0=53)
          annotation (Placement(transformation(extent={{-82,20},{-62,40}})));
        Components.PwTrasformerLTC_V pwTrasformerLTC_V(
          Snom=1200,
          Systembase=450,
          r0=1,
          Xn=0.0)
          annotation (Placement(transformation(extent={{28,22},{48,36}})));
        OpenIPSL.Electrical.Branches.Simulink.LTC.LTC lTC(
          Ymin=0.8,
          Ymax=1.1,
          delay1=20,
          delay2=10,
          positionNo=31)
          annotation (Placement(transformation(extent={{30,42},{60,62}})));
        OpenIPSL.Electrical.Buses.Bus FiveBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0)
          annotation (Placement(transformation(extent={{-8,20},{12,40}})));
        OpenIPSL.Electrical.Branches.PwLine pwLine2(
          G=0,
          B=0,
          R=0,
          X=0.018)
          annotation (Placement(transformation(extent={{-32,20},{-12,40}})));
        OpenIPSL.Electrical.Buses.Bus SixBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0)
          annotation (Placement(transformation(extent={{56,20},{76,40}})));
        OpenIPSL.Electrical.Loads.PSAT.ExponentialRecovery exponentialRecovery
          annotation (Placement(transformation(extent={{78,-10},{98,10}})));
        OpenIPSL.Electrical.Loads.PSAT.VoltDependant voltDependant
          annotation (Placement(transformation(extent={{-58,-44},{-38,-24}})));
      equation
        connect(infiniteBus.p, ThreeBus.p)
          annotation (Line(points={{-62,30},{-42,30}}, color={0,0,255}));
        connect(pwTrasformerLTC_V.u, lTC.r) annotation (Line(points={{44.9,32.9},
                {54,32.9},{54,53.9},{49.6,53.9}}, color={0,0,127}));
        connect(pwTrasformerLTC_V.rin, lTC.u) annotation (Line(points={{30.3,33},
                {26,33},{26,54.3},{30.2,54.3}}, color={0,0,127}));
        connect(pwTrasformerLTC_V.p, FiveBus.p) annotation (Line(points={{30.3,
                29.8},{18,29.8},{18,30},{2,30}},  color={0,0,255}));
        connect(ThreeBus.p, pwLine2.p)
          annotation (Line(points={{-42,30},{-31,30}}, color={0,0,255}));
        connect(FiveBus.p, pwLine2.n)
          annotation (Line(points={{2,30},{-13,30}},  color={0,0,255}));
        connect(SixBus.p, pwTrasformerLTC_V.n) annotation (Line(points={{66,30},
                {56,30},{56,29.9},{45.4,29.9}}, color={0,0,255}));
        connect(SixBus.p, exponentialRecovery.p)
          annotation (Line(points={{66,30},{88,30},{88,10}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartLoadDynamic;

      model PartMotorInfiniteBus
        OpenIPSL.Electrical.Buses.Bus TwoBus(
          V_b=20,
          V_0=1.01,
          angle_0=-10,
          P_0=300,
          Q_0=37)
          annotation (Placement(transformation(extent={{-36,14},{-16,34}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer(
          Vn=20,
          V_b=20,
          Sn=750,
          S_b=0,
          rT=0,
          xT=0.08)
          annotation (Placement(transformation(extent={{-2,14},{18,34}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{36,14},{56,34}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-100,78},{-40,98}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          displayPF=true,
          P_0=1200,
          Q_0=53,
          V_0=0.993,
          angle_0=-15.9)
          annotation (Placement(transformation(extent={{88,14},{68,34}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
          V_b=20,
          displayPF=true,
          P_0=300,
          Q_0=37,
          V_0=1.0058,
          angle_0=-12.2)
          annotation (Placement(transformation(extent={{-78,14},{-58,34}})));
      equation
        connect(TwoBus.p,twoWindingTransformer. p)
          annotation (Line(points={{-26,24},{-3,24}}, color={0,0,255}));
        connect(twoWindingTransformer.n,ThreeBus. p)
          annotation (Line(points={{19,24},{46,24}}, color={0,0,255}));
        connect(ThreeBus.p, infiniteBus.p)
          annotation (Line(points={{46,24},{68,24}}, color={0,0,255}));
        connect(TwoBus.p, infiniteBus1.p)
          annotation (Line(points={{-26,24},{-58,24}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartMotorInfiniteBus;

      model PartGeneratorVoltageSource
        OpenIPSL.Electrical.Buses.Bus TwoBus(
          V_b=20,
          V_0=1.01,
          angle_0=-10,
          P_0=300,
          Q_0=37)
          annotation (Placement(transformation(extent={{-32,14},{-12,34}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer twoWindingTransformer(
          rT=0,
          Sn=450,
          Vn=20,
          V_b=20,
          xT=8,
          m=1.06)
          annotation (Placement(transformation(extent={{-2,14},{18,34}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{20,14},{40,34}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
          annotation (Placement(transformation(extent={{-100,78},{-40,98}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{88,14},{68,34}})));
        Components.VoltageSource voltageSource(
          V_b=20,
          V_0=1.01,
          angle_0=-10,
          P_0=300,
          Q_0=37)
          annotation (Placement(transformation(extent={{-48,14},{-28,34}})));
        Modelica.Blocks.Sources.Constant VReal(k=0.9946558305)
          annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
        Modelica.Blocks.Sources.Constant VImag(k=-0.1753846594)
          annotation (Placement(transformation(extent={{-96,-8},{-76,12}})));
      equation
        connect(TwoBus.p,twoWindingTransformer. p)
          annotation (Line(points={{-22,24},{-3,24}}, color={0,0,255}));
        connect(twoWindingTransformer.n,ThreeBus. p)
          annotation (Line(points={{19,24},{30,24}}, color={0,0,255}));
        connect(ThreeBus.p, infiniteBus.p)
          annotation (Line(points={{30,24},{68,24}}, color={0,0,255}));
        connect(VReal.y,voltageSource. u1) annotation (Line(points={{-75,44},{-62,
                44},{-62,28},{-50,28}}, color={0,0,127}));
        connect(voltageSource.u2,VImag. y) annotation (Line(points={{-50,20},{-56,
                20},{-56,2},{-75,2}}, color={0,0,127}));
        connect(voltageSource.p, TwoBus.p)
          annotation (Line(points={{-27,24},{-22,24}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartGeneratorVoltageSource;

      model PartHolding
        OpenIPSL.Electrical.Machines.PSAT.MotorTypeI motorTypeI1(
          Hm=0.6,
          V_b=15,
          Rs=0.0186,
          Xs=1.98,
          Rr1=0.03,
          Xr1=1.962,
          Xm=1.92,
          Q_0=140,
          V_0=0.993,
          P_0=600,
          angle_0=-15.9)
          annotation (Placement(transformation(extent={{76,14},{56,34}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer(
          Sn=500,
          V_b=20,
          Vn=20,
          S_b=0,
          rT=0,
          xT=0.08)
          annotation (Placement(transformation(extent={{-86,0},{-66,20}})));
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={40,54})));
        Components.PwTrasformerLTC_V pwTrasformerLTC_V(
          Snom=1200,
          Systembase=450,
          r0=1,
          Xn=0.0)
          annotation (Placement(transformation(extent={{-32,46},{-12,60}})));
        OpenIPSL.Electrical.Branches.Simulink.LTC.LTC lTC(
          Ymin=0.8,
          Ymax=1.1,
          delay1=20,
          delay2=10,
          positionNo=31)
          annotation (Placement(transformation(extent={{-32,66},{-2,86}})));
        OpenIPSL.Electrical.Buses.Bus SixBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-15.2,
          P_0=1200,
          Q_0=0)
          annotation (Placement(transformation(extent={{-6,44},{14,64}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=15,
          displayPF=true,
          V_0=0.99299616,
          angle_0=-15.87392634852)
          annotation (Placement(transformation(extent={{12,-80},{-22,-52}})));
      equation
        connect(pwTrasformerLTC_V.u,lTC. r) annotation (Line(points={{-15.1,
                56.9},{-8,56.9},{-8,77.9},{-12.4,77.9}},
                                                  color={0,0,127}));
        connect(pwTrasformerLTC_V.rin,lTC. u) annotation (Line(points={{-29.7,
                57},{-36,57},{-36,78.3},{-31.8,78.3}},
                                                color={0,0,127}));
        connect(pwTrasformerLTC_V.p, FiveBus.p) annotation (Line(points={{-29.7,
                53.8},{-44,53.8},{-44,54},{-60,54}},
                                                  color={0,0,255}));
        connect(SixBus.p,lOADPQ. p)
          annotation (Line(points={{4,54},{30,54}},  color={0,0,255}));
        connect(SixBus.p,pwTrasformerLTC_V. n) annotation (Line(points={{4,54},{
                -6,54},{-6,53.9},{-14.6,53.9}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartHolding;

      model PartGeneratorTransformer
        OpenIPSL.Electrical.Buses.Bus bus(V_b=750)
          annotation (Placement(transformation(extent={{16,-10},{36,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1(V_b=750)
          annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer(
          Vn=20,
          V_b=20,
          Sn=750,
          S_b=750,
          rT=0,
          xT=0.08)
          annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-100,78},{-40,98}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          displayPF=true,
          V_b=20,
          V_0=1.04,
          angle_0=8.8,
          P_0=450,
          Q_0=90)
          annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
        OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ1(
          V_b=750,
          P_0=1200,
          Q_0=57)
          annotation (Placement(transformation(extent={{50,-56},{70,-36}})));
      equation
        connect(bus1.p,twoWindingTransformer. p)
          annotation (Line(points={{-30,0},{-13,0}}, color={0,0,255}));
        connect(bus.p,twoWindingTransformer. n)
          annotation (Line(points={{26,0},{9,0}}, color={0,0,255}));
        connect(bus1.p, infiniteBus.p)
          annotation (Line(points={{-30,0},{-46,0}}, color={0,0,255}));
        connect(lOADPQ1.p, bus.p) annotation (Line(points={{60,-36},{60,8},{40,
                8},{40,0},{26,0}},
                                color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartGeneratorTransformer;

      model PartTransformerLf3
        OpenIPSL.Electrical.Buses.Bus TwoBus(
          V_b=20,
          V_0=1.0058,
          angle_0=-12.2,
          P_0=600,
          Q_0=140)
          annotation (Placement(transformation(extent={{-38,14},{-18,34}})));
        OpenIPSL.Electrical.Buses.Bus ThreeBus(
          V_b=380,
          V_0=1.0455,
          angle_0=-12.7,
          P_0=-300,
          Q_0=-23)
          annotation (Placement(transformation(extent={{20,14},{40,34}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-100,78},{-40,98}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
          V_b=380,
          displayPF=true,
          V_0=1.0437,
          angle_0=-2.1)
          annotation (Placement(transformation(extent={{88,10},{54,38}})));
        OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
          V_b=20,
          P_0=300,
          Q_0=213,
          displayPF=true,
          V_0=1.01,
          angle_0=1.0)
          annotation (Placement(transformation(extent={{-80,10},{-48,38}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
          twoWindingTransformer1( Sn=500,
          V_b=20,
          Vn=20,
          S_b=750,
          rT=0,
          xT=0.08,
          m=1/1.04)
          annotation (Placement(transformation(extent={{-12,14},{8,34}})));
      equation
        connect(ThreeBus.p, infiniteBus.p)
          annotation (Line(points={{30,24},{54,24}}, color={0,0,255}));
        connect(TwoBus.p, infiniteBus1.p)
          annotation (Line(points={{-28,24},{-48,24}}, color={0,0,255}));
        connect(TwoBus.p, twoWindingTransformer1.p)
          annotation (Line(points={{-28,24},{-13,24}}, color={0,0,255}));
        connect(ThreeBus.p, twoWindingTransformer1.n)
          annotation (Line(points={{30,24},{9,24}}, color={0,0,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end PartTransformerLf3;

      package PartTesting

        package LF4
          model PartTransmissionLinesLF4B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.04,
              angle_0=0,
              P_0=800,
              Q_0=214,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0058,
              angle_0=-12.2,
              displayPF=true,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{90,22},{70,42}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,32},{62,32},{62,30},{54,30}},
                                                         color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF4B1B3;

          model PartLoadLF4
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0058,
              angle_0=-12.2,
              P_0=300.6,
              Q_0=192.2,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0024,
              angle_0=-13.3,
              P_0=500,
              Q_0=80)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p, ThreeBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF4;

          model PartMotorLF4
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=20,
              V_0=1.0058,
              angle_0=-12.2,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{-38,14},{-18,34}})));
            OpenIPSL.Electrical.Buses.Bus FourBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-300,
              Q_0=-23)
              annotation (Placement(transformation(extent={{20,14},{40,34}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,82},{-40,102}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              P_0=600,
              Q_0=140,
              displayPF=true,
              V_0=1.0058,
              angle_0=-12.2)
              annotation (Placement(transformation(extent={{-80,10},{-48,38}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=750,
              V_b=380,
              Vn=380,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1)
              annotation (Placement(transformation(extent={{-10,14},{10,34}})));
            OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
              Q_0=0,
              Hm=0.6,
              V_b=15,
              Sup=0,
              V_0=1.04,
              angle_0=-0.3,
              Rs=0.031,
              Xs=0.1,
              Rr1=0.05,
              Xr1=0.07,
              Xm=3.20,
              a=0.78,
              P_0=0.01333)
              annotation (Placement(transformation(extent={{98,14},{78,34}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=15,
              P_0=0,
              Q_0=-202) annotation (Placement(transformation(extent={{-10,-10},{10,
                      10}},
                  rotation=90,
                  origin={88,-2})));
          equation
            connect(ThreeBus.p, infiniteBus1.p)
              annotation (Line(points={{-28,24},{-48,24}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-28,24},{-11,24}}, color={0,0,255}));
            connect(FourBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{30,24},{11,24}}, color={0,0,255}));
            connect(FourBus.p, motorTypeIII.p)
              annotation (Line(points={{30,24},{78,24}}, color={0,0,255}));
            connect(lOADPQ.p, motorTypeIII.p) annotation (Line(points={{78,-2},{50,
                    -2},{50,24},{78,24}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartMotorLF4;

          model PartTransmissionLinesLF4B3B5
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,20},{14,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0058076,
              angle_0=-12.199912982449,
              P_0=800,
              Q_0=214,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0024174,
              angle_0=-13.336538635023,
              displayPF=true,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{94,20},{74,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,30},{-42,30}},
                                        color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,30},{54,30}},
                                      color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{74,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF4B3B5;

          model PartTransformerLF4B2B3
            OpenIPSL.Electrical.Buses.Bus TwoBus(
              V_b=20,
              V_0=1.0058,
              angle_0=-12.2,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{-38,14},{-18,34}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-300,
              Q_0=-23)
              annotation (Placement(transformation(extent={{20,14},{40,34}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,82},{-40,102}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              displayPF=true,
              V_0=1.0058076,
              angle_0=-12.2)
              annotation (Placement(transformation(extent={{88,10},{54,38}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=20,
              P_0=300,
              Q_0=213,
              displayPF=true,
              V_0=1,
              angle_0=-9.4)
              annotation (Placement(transformation(extent={{-80,10},{-48,38}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-10,14},{10,34}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{30,24},{54,24}}, color={0,0,255}));
            connect(TwoBus.p, infiniteBus1.p)
              annotation (Line(points={{-28,24},{-48,24}}, color={0,0,255}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-28,24},{-11,24}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{30,24},{11,24}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransformerLF4B2B3;

          model PartGeneratorLF4
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,78},{-40,98}})));
            OpenIPSL.Electrical.Buses.Bus bus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus bus1(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.005876,
              angle_0=-12.2,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=300,
              M=7,
              Sn=500,
              w(fixed=true),
              D=1,
              V_0=1,
              angle_0=-9.4,
              Q_0=213,
              ra=0.001,
              x1d=1.92)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-12},{2,8}})));
          equation
            connect(bus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, bus1.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(bus1.p, twoWindingTransformer1.p) annotation (Line(points={{-40,
                    0},{-30,0},{-30,-2},{-19,-2}}, color={0,0,255}));
            connect(bus.p, twoWindingTransformer1.n) annotation (Line(points={{26,0},
                    {14,0},{14,-2},{3,-2}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF4;
        end LF4;

        package LF1
          model PartTransmissionLinesLF1B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.05999999,
              angle_0=0,
              P_0=-350,
              Q_0=-16.681575,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0682527,
              angle_0=4.8995092990333919,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{90,20},{70,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF1B1B3;

          model PartLoadLF1
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=450)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0683,
              angle_0=4.9,
              P_0=300.6,
              Q_0=192.2,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0675,
              angle_0=4.7,
              P_0=100,
              Q_0=20)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p,ThreeBus. p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF1;

          model PartTransformerLF1B3B4
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=20,
              V_0=1.0058,
              angle_0=-12.2,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{-38,14},{-18,34}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,82},{-40,102}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              P_0=0,
              Q_0=0,
              displayPF=true,
              V_0=1.0683,
              angle_0=4.9)
              annotation (Placement(transformation(extent={{-80,10},{-48,38}})));
          equation
            connect(ThreeBus.p, infiniteBus1.p)
              annotation (Line(points={{-28,24},{-48,24}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransformerLF1B3B4;

          model PartTransmissionLinesLF1B3B5
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,20},{14,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0683,
              angle_0=4.9,
              P_0=0,
              Q_0=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0675,
              angle_0=4.7,
              displayPF=true,
              P_0=100,
              Q_0=20)
              annotation (Placement(transformation(extent={{94,20},{74,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine3.p, ThreeBus.p)
              annotation (Line(points={{-5,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.n, FiveBus.p)
              annotation (Line(points={{13,30},{54,30}}, color={0,0,255}));
            connect(infiniteBus.p, ThreeBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, FiveBus.p)
              annotation (Line(points={{74,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF1B3B5;

          model PartTransformerLF1B2B3
            OpenIPSL.Electrical.Buses.Bus TwoBus(
              V_b=20,
              V_0=1.0058,
              angle_0=-12.2,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{-38,14},{-18,34}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-300,
              Q_0=-23)
              annotation (Placement(transformation(extent={{20,14},{40,34}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              P_0=0,
              Q_0=0,
              displayPF=true,
              V_0=1.07,
              angle_0=4.9)
              annotation (Placement(transformation(extent={{88,10},{54,38}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=20,
              P_0=450,
              Q_0=99,
              displayPF=true,
              V_0=1.04,
              angle_0=8.8)
              annotation (Placement(transformation(extent={{-80,10},{-48,38}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=0.988)
              annotation (Placement(transformation(extent={{-10,14},{10,34}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{30,24},{54,24}}, color={0,0,255}));
            connect(TwoBus.p, infiniteBus1.p)
              annotation (Line(points={{-28,24},{-48,24}}, color={0,0,255}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-28,24},{-11,24}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{30,24},{11,24}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransformerLF1B2B3;

          model PartGeneratorLF1
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-178,78},{-118,98}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0683,
              angle_0=4.9,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=450,
              M=7,
              Sn=500,
              w(fixed=true),
              D=1,
              V_0=1.04,
              angle_0=8.8,
              Q_0=99,
              ra=0.001,
              x1d=0.4148)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, TwoBus.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-40,0},{-19,0}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{26,0},{3,0}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF1;
        end LF1;

        package LF2
          model PartTransmissionLinesLF2B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.05999999,
              angle_0=0,
              P_0=150,
              Q_0=66,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0433,
              angle_0=-0.7,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{90,20},{70,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF2B1B3;

          model PartLoadLF2
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0443,
              angle_0=-0.7,
              P_0=0,
              Q_0=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0411,
              angle_0=-1.6,
              P_0=400,
              Q_0=80)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p,ThreeBus. p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF2;

          model PartGeneratorLF2
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-178,78},{-118,98}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0443,
              angle_0=-0.7,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=350,
              M=7,
              Sn=500,
              w(fixed=true),
              D=1,
              V_0=1.01,
              angle_0=2.5,
              Q_0=47,
              ra=0.001,
              x1d=0.4148)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-12},{2,8}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, TwoBus.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(TwoBus.p, twoWindingTransformer1.p) annotation (Line(points={
                    {-40,0},{-30,0},{-30,-2},{-19,-2}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n) annotation (Line(points=
                   {{26,0},{14,0},{14,-2},{3,-2}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF2;
        end LF2;

        package LF3
          model PartTransmissionLinesLF3B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.05999999,
              angle_0=0,
              P_0=150,
              Q_0=66,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0443048,
              angle_0=-2.1,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{90,20},{70,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF3B1B3;

          model PartLoadLF3
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0437,
              angle_0=-2.1,
              P_0=0,
              Q_0=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0404,
              angle_0=-3.2,
              P_0=500,
              Q_0=80)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p,ThreeBus. p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF3;

          model PartGeneratorLF3
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-178,78},{-118,98}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0437,
              angle_0=-2.1,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=350,
              M=7,
              Sn=500,
              w(fixed=true),
              D=1,
              V_0=1.01,
              angle_0=1.0,
              Q_0=50,
              ra=0.001,
              x1d=0.4148)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, TwoBus.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-40,0},{-19,0}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{26,0},{3,0}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF3;
        end LF3;

        package LF5
          model PartTransmissionLinesLF5B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.08,
              angle_0=0,
              P_0=150,
              Q_0=66,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{90,20},{70,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF5B1B3;

          model PartLoadLF5
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=0,
              Q_0=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0404,
              angle_0=-3.2,
              P_0=1200,
              Q_0=0) annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p,ThreeBus. p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF5;

          model PartGeneratorLF5
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-178,78},{-118,98}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=300,
              M=7,
              Sn=500,
              w(fixed=true),
              D=15,
              V_0=1.01,
              angle_0=-10,
              Q_0=37,
              ra=0.001,
              x1d=0.4148)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, TwoBus.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-40,0},{-19,0}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{26,0},{3,0}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF5;
        end LF5;

        package LF6
          model PartTransmissionLinesLF6B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.08,
              angle_0=0,
              P_0=150,
              Q_0=66,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0166,
              angle_0=-17.6,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{90,20},{70,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF6B1B3;

          model PartLoadLF6
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0166,
              angle_0=-17.6,
              P_0=0,
              Q_0=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0404,
              angle_0=-3.2,
              P_0=1500,
              Q_0=150)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p,ThreeBus. p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF6;

          model PartGeneratorLF6
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-178,78},{-118,98}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=300,
              M=7,
              Sn=500,
              w(fixed=true),
              D=15,
              V_0=1.01,
              angle_0=-14.8,
              Q_0=213,
              ra=0.001,
              x1d=0.4148)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, TwoBus.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-40,0},{-19,0}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{26,0},{3,0}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF6;
        end LF6;

        package LF7
          model PartTransmissionLinesLF7B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.08,
              angle_0=0,
              P_0=150,
              Q_0=66,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0205,
              angle_0=-15.3,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{90,20},{70,40}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,30},{54,30}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF7B1B3;

          model PartLoadLF7
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0205,
              angle_0=-15.3,
              P_0=0,
              Q_0=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0404,
              angle_0=-3.2,
              P_0=1500,
              Q_0=150)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p,ThreeBus. p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF7;

          model PartGeneratorLF7
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-178,78},{-118,98}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0205,
              angle_0=-15.3,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=450,
              M=7,
              Sn=500,
              w(fixed=true),
              D=15,
              V_0=1.01,
              angle_0=-11.1,
              Q_0=198,
              ra=0.001,
              x1d=0.4148)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
          equation
            connect(ThreeBus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, TwoBus.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-148,32},{-148,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-40},{-148,-40},{-148,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(TwoBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-40,0},{-19,0}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{26,0},{3,0}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF7;
        end LF7;

        package LF8
          model PartTransmissionLinesLF8B1B3
            OpenIPSL.Electrical.Buses.Bus OneBus(
              angle_0=0,
              V_b=380,
              V_0=1.08,
              P_0=450,
              Q_0=118)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-450,
              Q_0=-15)
              annotation (Placement(transformation(extent={{44,20},{64,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.4143333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,46},{14,66}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine3(
              R=0,
              X=0.414333333,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-6,-4},{14,16}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.08,
              angle_0=0,
              P_0=800,
              Q_0=214,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              V_0=1.0117,
              angle_0=-17.7,
              displayPF=true,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{90,22},{70,42}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
          equation
            connect(pwLine2.p, OneBus.p) annotation (Line(points={{-5,56},{-18,56},
                    {-18,30},{-42,30}}, color={0,0,255}));
            connect(pwLine3.p, OneBus.p) annotation (Line(points={{-5,6},{-18,6},{
                    -18,30},{-42,30}},  color={0,0,255}));
            connect(pwLine2.n, ThreeBus.p) annotation (Line(points={{13,56},{28,56},
                    {28,30},{54,30}}, color={0,0,255}));
            connect(pwLine3.n, ThreeBus.p) annotation (Line(points={{13,6},{28,6},{
                    28,30},{54,30}},  color={0,0,255}));
            connect(infiniteBus.p, OneBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(infiniteBus1.p, ThreeBus.p)
              annotation (Line(points={{70,32},{62,32},{62,30},{54,30}},
                                                         color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartTransmissionLinesLF8B1B3;

          model PartLoadLF8
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,80},{-40,100}})));
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=1200,
              Q_0=53)
              annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0117,
              angle_0=-17.7,
              P_0=300.6,
              Q_0=192.2,
              displayPF=true)
              annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
            OpenIPSL.Electrical.Buses.Bus FiveBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-15.2,
              P_0=1200,
              Q_0=0)
              annotation (Placement(transformation(extent={{0,20},{20,40}})));
            OpenIPSL.Electrical.Branches.PwLine pwLine2(
              R=0,
              X=0.029999989612,
              G=0,
              B=0,
              displayPF=true)
              annotation (Placement(transformation(extent={{-26,20},{-6,40}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=380,
              V_0=1.0024,
              angle_0=-13.3,
              P_0=900,
              Q_0=50)
                     annotation (Placement(transformation(
                  extent={{-17,-17},{17,17}},
                  rotation=90,
                  origin={45,31})));
          equation
            connect(infiniteBus.p, ThreeBus.p)
              annotation (Line(points={{-60,30},{-42,30}}, color={0,0,255}));
            connect(ThreeBus.p, pwLine2.p)
              annotation (Line(points={{-42,30},{-25,30}}, color={0,0,255}));
            connect(FiveBus.p, pwLine2.n)
              annotation (Line(points={{10,30},{-7,30}}, color={0,0,255}));
            connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{10,30},{22,30},
                    {22,31},{28,31}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartLoadLF8;

          model PartGeneratorLF4
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,78},{-40,98}})));
            OpenIPSL.Electrical.Buses.Bus bus(V_b=750)
              annotation (Placement(transformation(extent={{16,-10},{36,10}})));
            OpenIPSL.Electrical.Buses.Bus bus1(V_b=750)
              annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
              V_b=380,
              V_0=1.0117,
              angle_0=-17.7,
              displayPF=true,
              P_0=0,
              Q_0=0)
              annotation (Placement(transformation(extent={{94,-24},{48,24}})));
            OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
              Vn=20,
              V_b=20,
              P_0=300,
              M=7,
              Sn=500,
              w(fixed=true),
              D=1,
              V_0=1,
              angle_0=-14.8,
              Q_0=177,
              ra=0.001,
              x1d=1.92)
              annotation (Placement(transformation(extent={{-114,-22},{-70,22}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=500,
              V_b=20,
              Vn=20,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1/1.04)
              annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
          equation
            connect(bus.p, infiniteBus.p)
              annotation (Line(points={{26,0},{48,0}}, color={0,0,255}));
            connect(order2_1.p, bus1.p)
              annotation (Line(points={{-70,0},{-40,0}}, color={0,0,255}));
            connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{-109.6,
                    24.2},{-109.6,32},{-136,32},{-136,11},{-118.4,11}}, color={0,0,
                    127}));
            connect(order2_1.pm0, order2_1.pm) annotation (Line(points={{-109.6,
                    -24.2},{-109.6,-26},{-132,-26},{-132,-11},{-118.4,-11}}, color=
                    {0,0,127}));
            connect(bus1.p, twoWindingTransformer1.p) annotation (Line(points={{-40,0},
                    {-21,0}},                      color={0,0,255}));
            connect(bus.p, twoWindingTransformer1.n) annotation (Line(points={{26,0},{
                    1,0}},                  color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                      -180,-120},{100,100}})), Diagram(coordinateSystem(
                    preserveAspectRatio=false, extent={{-180,-120},{100,100}})));
          end PartGeneratorLF4;

          model PartMotorLF8
            OpenIPSL.Electrical.Buses.Bus ThreeBus(
              V_b=20,
              V_0=1.0058,
              angle_0=-12.2,
              P_0=600,
              Q_0=140)
              annotation (Placement(transformation(extent={{-38,14},{-18,34}})));
            OpenIPSL.Electrical.Buses.Bus FourBus(
              V_b=380,
              V_0=1.0455,
              angle_0=-12.7,
              P_0=-300,
              Q_0=-23)
              annotation (Placement(transformation(extent={{20,14},{40,34}})));
            inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
              annotation (Placement(transformation(extent={{-100,82},{-40,102}})));
            OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus1(
              V_b=380,
              P_0=600,
              Q_0=140,
              displayPF=true,
              V_0=1.0117,
              angle_0=-17.7)
              annotation (Placement(transformation(extent={{-80,10},{-48,38}})));
            OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
              twoWindingTransformer1(
              Sn=750,
              V_b=380,
              Vn=380,
              S_b=750,
              rT=0,
              xT=0.08,
              m=1)
              annotation (Placement(transformation(extent={{-10,14},{10,34}})));
            OpenIPSL.Electrical.Machines.PSAT.MotorTypeIII motorTypeIII(
              Q_0=0,
              Hm=0.6,
              V_b=15,
              Sup=0,
              V_0=0.9990,
              angle_0=-21.3,
              Rs=0.031,
              Xs=0.1,
              Rr1=0.05,
              Xr1=0.07,
              Xm=3.20,
              a=0.78,
              P_0=0.01333)
              annotation (Placement(transformation(extent={{98,14},{78,34}})));
            OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
              V_b=15,
              P_0=0,
              Q_0=-202) annotation (Placement(transformation(extent={{-10,-10},{10,
                      10}},
                  rotation=90,
                  origin={88,-2})));
          equation
            connect(ThreeBus.p, infiniteBus1.p)
              annotation (Line(points={{-28,24},{-48,24}}, color={0,0,255}));
            connect(ThreeBus.p, twoWindingTransformer1.p)
              annotation (Line(points={{-28,24},{-11,24}}, color={0,0,255}));
            connect(FourBus.p, twoWindingTransformer1.n)
              annotation (Line(points={{30,24},{11,24}}, color={0,0,255}));
            connect(FourBus.p, motorTypeIII.p)
              annotation (Line(points={{30,24},{78,24}}, color={0,0,255}));
            connect(lOADPQ.p, motorTypeIII.p) annotation (Line(points={{78,-2},{50,
                    -2},{50,24},{78,24}}, color={0,0,255}));
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end PartMotorLF8;
        end LF8;

        model AIOSHYGOV
          OpenIPSL.Electrical.Buses.Bus TwoBus(V_b=750) annotation (Placement(
                transformation(extent={{-258,-104},{-234,-80}})));
          OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer
            twoWindingTransformer1(
            Sn=500,
            V_b=20,
            Vn=20,
            S_b=750,
            rT=0,
            xT=0.08,
            m=1/1.04)
            annotation (Placement(transformation(extent={{-176,-102},{-156,-82}})));
          OpenIPSL.Electrical.Buses.Bus ThreeBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-12.7,
            P_0=1200,
            Q_0=53)
            annotation (Placement(transformation(extent={{-116,-106},{-88,-78}})));
          OpenIPSL.Electrical.Buses.Bus FiveBus(
            V_b=380,
            V_0=1.0455,
            angle_0=-15.2,
            P_0=1200,
            Q_0=0)
            annotation (Placement(transformation(extent={{-6,-104},{20,-78}})));
          OpenIPSL.Electrical.Loads.PSAT.LOADPQ lOADPQ(
            V_b=380,
            V_0=pF1_1.voltage.PQLoadV_0,
            angle_0=pF1_1.voltage.PQLoadangle_0,
            P_0=pF1_1.power.PQLoadP_0,
            Q_0=pF1_1.power.PQLoadQ_0)
                   annotation (Placement(transformation(
                extent={{-22,-22},{22,22}},
                rotation=90,
                origin={84,-90})));
          inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
            annotation (Placement(transformation(extent={{-474,50},{-408,76}})));
          OpenIPSL.Electrical.Buses.InfiniteBus infiniteBus(
            V_b=380,
            displayPF=true,
            V_0=pF1_1.voltage.InfiniteBusV_0,
            angle_0=pF1_1.voltage.InfiniteBusangle_0,
            P_0=pF1_1.power.InfiniteBusP_0,
            Q_0=pF1_1.power.InfiniteBusQ_0)
            annotation (Placement(transformation(extent={{-476,-10},{-438,14}})));
          OpenIPSL.Electrical.Buses.Bus OneBus(
            angle_0=0,
            V_b=380,
            V_0=1.08,
            P_0=450,
            Q_0=118)
            annotation (Placement(transformation(extent={{-432,-10},{-410,12}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine3(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-322,-44},{-296,-18}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine2(
            R=0,
            X=0.029999989612,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-64,-104},{-40,-80}})));
          OpenIPSL.Electrical.Branches.PwLine pwLine1(
            R=0,
            X=0.414333333,
            G=0,
            B=0,
            displayPF=true)
            annotation (Placement(transformation(extent={{-322,12},{-296,38}})));
          AIOSModel.Data.SystemData.SystemData.PF1 pF1_1 annotation (Placement(
                transformation(extent={{-396,58},{-376,78}})));
          Parts.GeneratorHYGOV generatorHYGOV(
            V_b=20,
            V_0=pF1_1.voltage.GeneratorV_0,
            angle_0=pF1_1.voltage.Generatorangle_0,
            P_0=pF1_1.power.GeneratorP_0,
            Q_0=pF1_1.power.GeneratorQ_0,
            M_b=750) annotation (Placement(transformation(extent={{-448,-124},{
                    -372,-78}})));
        equation
          connect(TwoBus.p, twoWindingTransformer1.p)
            annotation (Line(points={{-246,-92},{-177,-92}}, color={0,0,255}));
          connect(twoWindingTransformer1.n, ThreeBus.p)
            annotation (Line(points={{-155,-92},{-102,-92}},color={0,0,255}));
          connect(ThreeBus.p, pwLine2.p)
            annotation (Line(points={{-102,-92},{-62.8,-92}},
                                                           color={0,0,255}));
          connect(FiveBus.p, pwLine2.n)
            annotation (Line(points={{7,-91},{-10,-91},{-10,-92},{-41.2,-92}},
                                                         color={0,0,255}));
          connect(infiniteBus.p, OneBus.p)
            annotation (Line(points={{-438,2},{-436,2},{-436,1},{-421,1}},
                                                         color={0,0,255}));
          connect(pwLine3.n, pwLine1.n) annotation (Line(points={{-297.3,-31},{
                  -250,-31},{-250,25},{-297.3,25}}, color={0,0,255}));
          connect(pwLine1.p, pwLine3.p) annotation (Line(points={{-320.7,25},{
                  -392,25},{-392,-31},{-320.7,-31}}, color={0,0,255}));
          connect(OneBus.p, pwLine3.p) annotation (Line(points={{-421,1},{-412,1},
                  {-412,0},{-392,0},{-392,-31},{-320.7,-31}}, color={0,0,255}));
          connect(pwLine1.n, ThreeBus.p) annotation (Line(points={{-297.3,25},{
                  -250,25},{-250,0},{-102,0},{-102,-92}}, color={0,0,255}));
          connect(TwoBus.p, generatorHYGOV.pwPin) annotation (Line(points={{
                  -246,-92},{-278,-92},{-278,-97.1667},{-370.48,-97.1667}},
                color={0,0,255}));
          connect(FiveBus.p, lOADPQ.p) annotation (Line(points={{7,-91},{62,-91},
                  {62,-90}}, color={0,0,255}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-480,
                    -140},{140,80}})),      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-480,-140},{140,80}})));
        end AIOSHYGOV;

        model PartHold
          OpenIPSL.Electrical.Controls.PSAT.PSS.PSSTypeII pSSTypeII(
            vsmax=999,
            vsmin=-999,
            T1=1,
            T2=10,
            T3=0,
            T4=0,
            Kw=190,
            Tw=0.001)
            annotation (Placement(transformation(extent={{-26,6},{-6,26}})));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
              Diagram(coordinateSystem(preserveAspectRatio=false)));
        end PartHold;
      end PartTesting;

      model GeneratorHYGOV
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
          annotation (Placement(transformation(extent={{94,-10},{114,10}})));
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
              origin={2,-90})));
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
        OpenIPSL.Electrical.Controls.PSSE.TG.HYGOV hYGOV(VELM=0.02, G_MAX=0.415)
          annotation (Placement(transformation(extent={{6,2},{22,12}})));
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
          annotation (Line(points={{62,0},{104,0}}, color={0,0,255}));
        connect(const2.y, eSST1A.VUEL2) annotation (Line(points={{2,-79},{2,-60},
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
        connect(hYGOV.SPEED, gENSAL.SPEED) annotation (Line(points={{6.66667,
                10.3333},{2,10.3333},{2,16},{66,16},{66,6},{63,6},{63,7}},
              color={0,0,127}));
        connect(gENSAL.PMECH0, hYGOV.PMECH0) annotation (Line(points={{63,5},{
                68,5},{68,18},{0,18},{0,4.22222},{6.66667,4.22222}}, color={0,0,
                127}));
        connect(hYGOV.PMECH, gENSAL.PMECH) annotation (Line(points={{22.3333,
                7.55556},{32,7.55556},{32,5},{40,5}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -100,-140},{100,100}})),                             Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{
                  100,100}})));
      end GeneratorHYGOV;

      model HYGOV
        OpenIPSL.Electrical.Controls.PSSE.TG.HYGOV hYGOV(VELM=0.02, G_MAX=0.415)
          annotation (Placement(transformation(extent={{-14,-18},{34,18}})));
        OpenIPSL.Electrical.Controls.PSSE.TG.TGOV1 tGOV1_1(
          R=0.05,
          D_t=0.2,
          V_MIN=0,
          V_MAX=0.415)
          annotation (Placement(transformation(extent={{-16,-68},{32,-30}})));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end HYGOV;

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

      model PSATGeneratorTGOV
        OpenIPSL.Interfaces.PwPin pwPin
          annotation (Placement(transformation(extent={{94,-12},{114,8}}),
              iconTransformation(extent={{94,-12},{114,8}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=750)
          annotation (Placement(transformation(extent={{-98,78},{-38,98}})));
      //   parameter OpenIPSL.Types.VoltageKilo V_b=400 "Base voltage of the bus";
      //   parameter Modelica.SIunits.PerUnit V_0=1 "Voltage magnitude (pu)";
      //   parameter Modelica.SIunits.Conversions.NonSIunits.Angle_deg angle_0=0
      //     "Voltage angle";
      //   parameter OpenIPSL.Types.ActivePowerMega P_0=1 "Active power";
      //   parameter OpenIPSL.Types.ReactivePowerMega Q_0=0 "Reactive power";
      //   parameter Real M_b=460 "Machine base power (MVA)";
        OpenIPSL.Electrical.Machines.PSAT.Order2 order2_1(
          V_b=20,
          Sn=500,
          Vn=20,
          ra=0,
          x1d=0.4148,
          M=7,
          D=1,
          V_0=V_0,
          angle_0=angle_0,
          P_0=P_0,
          Q_0=Q_0)
          annotation (Placement(transformation(extent={{42,2},{62,22}})));
        parameter Modelica.SIunits.PerUnit V_0
          "Voltage magnitude (pu)";
        parameter Modelica.SIunits.Conversions.NonSIunits.Angle_deg angle_0
          "Voltage angle";
        parameter OpenIPSL.Types.ActivePowerMega P_0
          "Active power";
        parameter OpenIPSL.Types.ReactivePowerMega Q_0
          "Reactive power";
        OpenIPSL.Electrical.Controls.PSAT.TG.TGtypeII tGtypeII(
          R=0.05,
          pmax0=0.7,
          pmin0=0,
          S_b=750,
          Sn=460,
          Ts=0.5,
          T3=0.05)
          annotation (Placement(transformation(extent={{-2,-52},{18,-32}})));
      equation
        connect(pwPin, order2_1.p) annotation (Line(points={{104,-2},{76,-2},{
                76,12},{62,12}},
                         color={0,0,255}));
        connect(tGtypeII.pm, order2_1.pm) annotation (Line(points={{19,-42},{24,
                -42},{24,7},{40,7}}, color={0,0,127}));
        connect(order2_1.pm0, tGtypeII.pm0) annotation (Line(points={{44,1},{44,
                -14},{8,-14},{8,-30}}, color={0,0,127}));
        connect(order2_1.w, tGtypeII.w) annotation (Line(points={{63,21},{72,21},
                {72,-58},{-10,-58},{-10,-42},{-4,-42}}, color={0,0,127}));
        connect(order2_1.vf0, order2_1.vf) annotation (Line(points={{44,23},{44,
                32},{24,32},{24,17},{40,17}}, color={0,0,127}));
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
      end PSATGeneratorTGOV;
    end Parts;
  end AIOS;
  annotation (uses(iPSL(version="1.1.0"),
      Modelica(version="3.2.2"),
      Complex(version="3.2.2"),
      Tutorial(version="1"),
      OpenIPSL(version="2.0.0-dev"),
      Modelica_Synchronous(version="0.93.0")));
end RelayCoordination;

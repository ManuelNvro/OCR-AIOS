within OpenIPSL.Examples.Controls.PSAT.TG;
partial model TGTestBase
  OpenIPSL.Electrical.Loads.PSAT.LOADPQ pwLoadPQ1(
    P_0=0.08,
    Q_0=0.06,
    V_0=1,
    angle_0=0) annotation (Placement(visible=true, transformation(
        origin={90.0709,0},
        extent={{-5.92912,-6.00002},{5.92911,6.00001}},
        rotation=0)));
  OpenIPSL.Electrical.Branches.PwLine pwLineFault(
    B=0.001/2,
    G=0,
    R=0.01,
    X=0.1,
    t1=8,
    t2=8.1) annotation (Placement(visible=true, transformation(
        origin={70,30},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  OpenIPSL.Electrical.Branches.PwLine pwLine3(
    B=0.001/2,
    G=0,
    R=0.01,
    X=0.1) annotation (Placement(visible=true, transformation(
        origin={70,10},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  OpenIPSL.Electrical.Branches.PwLine pwLine4(
    B=0.001/2,
    G=0,
    R=0.01,
    X=0.1) annotation (Placement(visible=true, transformation(
        origin={70,-10},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  OpenIPSL.Electrical.Loads.PSAT.LOADPQ pwLoadPQ2(
    P_0=0.08,
    Q_0=0.06,
    V_0=1,
    angle_0=0) annotation (Placement(visible=true, transformation(
        origin={90,-26},
        extent={{-6,-6},{6,6}},
        rotation=0)));
  OpenIPSL.Electrical.Branches.PwLine pwLine1(
    B=0.001/2,
    G=0,
    R=0.01,
    X=0.1,
    displayPF=false) annotation (Placement(visible=true, transformation(
        origin={46,10},
        extent={{-10.0,-10.0},{10.0,10.0}},
        rotation=0)));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    B=0.001/2,
    G=0,
    R=0.01,
    X=0.1) annotation (Placement(visible=true, transformation(
        origin={46,-10},
        extent={{-10.0,-10.0},{10.0,10.0}},
        rotation=0)));
  OpenIPSL.Electrical.Events.PwFaultPQ pwFault(
    X=0.001,
    t1=3,
    t2=3.1,
    R=10) annotation (Placement(transformation(extent={{76,-36},{64,-24}})));
  parameter Real p0=0.160352698692006 "Power flow, node active power";
  inner OpenIPSL.Electrical.SystemBase SysData
    annotation (Placement(transformation(extent={{-100,80},{-60,100}})));
  Electrical.Machines.PSAT.Order3 gen(
    P_0=0.16041,
    Q_0=0.12012,
    V_b=400,
    V_0=1,
    angle_0=0,
    Sn=20,
    Vn=400,
    ra=0.001,
    x1d=0.302,
    M=10,
    D=0,
    xd=1.9,
    T1d0=8,
    xq=1.7) annotation (Placement(visible=true, transformation(
        origin={0,0},
        extent={{-20,-20},{20,20}},
        rotation=0)));
equation
  connect(pwLine1.n, pwLine3.p) annotation (Line(
      points={{55,10},{61,10}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(pwLine2.n,pwLine4. p) annotation (Line(
      points={{55,-10},{61,-10}},
      color={0,0,255},
      smooth=Smooth.None));

  connect(pwLine3.p, pwLineFault.p) annotation (Line(points={{61,10},{61,30}}, color={0,0,255}));
  connect(pwLineFault.n, pwLine3.n) annotation (Line(points={{79,30},{79,10}}, color={0,0,255}));
  connect(pwFault.p, pwLine4.n) annotation (Line(points={{77,-30},{79,-30},{79,-10}}, color={0,0,255}));
  connect(pwLoadPQ2.p, pwLine4.n) annotation (Line(points={{90,-20},{90,-10},{79,-10}}, color={0,0,255}));
  connect(pwLoadPQ1.p, pwLine3.n) annotation (Line(points={{90.0709,6},{90,6},{90,10},{79,10}}, color={0,0,255}));
  connect(pwLine1.n, pwLine2.n) annotation (Line(points={{55,10},{55,10},{55,-10}}, color={0,0,255}));
  connect(gen.vf0, gen.vf) annotation (Line(points={{-16,22},{-16,28},{-32,28},{-32,10},{-24,10}}, color={0,0,127}));
  connect(gen.p, pwLine1.p) annotation (Line(points={{20,0},{37,0},{37,10}}, color={0,0,255}));
  connect(gen.p, pwLine2.p) annotation (Line(points={{20,0},{37,0},{37,-10}}, color={0,0,255}));
  annotation (
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}},
          preserveAspectRatio=false)),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}})),
    Documentation);
end TGTestBase;

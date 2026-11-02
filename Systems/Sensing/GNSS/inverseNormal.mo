within fire_modelica_models.Systems.Sensing.GNSS;
function inverseNormal "Small normal matrix inverse with relative pivot/rank rejection"
  input Real A[4,4];
  input Real tolerance=1e-10;
  output Real inverse[4,4];
  output Boolean valid;
protected
  Real augmented[4,8];
  Real row[8];
  Real pivot;
  Real factor;
  Real scale;
  Integer selected;
algorithm
  augmented:=cat(2,A,identity(4));
  scale:=max(abs(A));
  valid:=scale>0;
  inverse:=zeros(4,4);
  if not valid then
    return;
  end if;
  for k in 1:4 loop
    selected:=k;
    for j in k+1:4 loop
      if abs(augmented[j,k])>abs(augmented[selected,k]) then
        selected:=j;
      end if;
    end for;
    if abs(augmented[selected,k])<=tolerance*scale then
      valid:=false;
      return;
    end if;
    row:=augmented[k,:];
    augmented[k,:]:=augmented[selected,:];
    augmented[selected,:]:=row;
    pivot:=augmented[k,k];
    augmented[k,:]:=augmented[k,:]/pivot;
    for j in 1:4 loop
      if j<>k then
        factor:=augmented[j,k];
        augmented[j,:]:=augmented[j,:]-factor*augmented[k,:];
      end if;
    end for;
  end for;
  inverse:=augmented[:,5:8];
end inverseNormal;

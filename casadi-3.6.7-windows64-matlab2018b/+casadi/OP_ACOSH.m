function v = OP_ACOSH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 58);
  end
  v = vInitialized;
end

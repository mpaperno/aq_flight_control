function incbuild() {
  var file = "buildnum.h";
  var buildNumText = "#define BUILDNUMBER ";
  var s = CWSys.readStringFromFile(file);
  var bs = s.split("\n", 2);
  var n = eval(bs[0].substring(buildNumText.length)) + 1;

  CWSys.writeStringToFile(file, buildNumText + n + '\n' + bs[1] + '\n');
}

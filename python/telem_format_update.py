savetypes = {"int":"%d",
         "unsigned int":"%d",
         "long":"%d",
         "unsigned long":"%d",
         "float":"%f",
         "char":"%s"}
         
unpacktypes = {"int":"h",
         "unsigned int":"H",
         "long":"l",
         "unsigned long":"L",
         "float":"f",
         "char":"s"}
         
         
headername = "..\lib\or_telem.h"
file = open(headername)

line = file.readline().strip()
while line != "//START_PARSE":
    line = file.readline().strip()
    
decls = []    

line = file.readline().strip()
while line != "//END_PARSE":
    decls.append(line)
    line = file.readline().strip()

print decls

savestring = []
for item in decls:
    temp = item.strip(';')
    temp = item.split(' ')
    numThisType = len(temp) - 1
    savestring.append(savetypes[temp[0]])
    
print savestring
# ----- Imports ----- #
import os
import fnmatch
import re
from datetime import date

# ----- Methods ----- #

# Method to find and replace some text in files inside a directory
def findReplaceText(directory, findRegex, substituteExpr, filePattern):
    for path, dirs, files in os.walk(os.path.abspath(directory)):
        for filename in fnmatch.filter(files, filePattern):
            filepath = os.path.join(path, filename)
            with open(filepath) as f:
                s = f.read()
            s = re.sub(findRegex, substituteExpr, s)
            with open(filepath, "w") as f:
                f.write(s)
      
# ----- Code ----- # 

# Read new version number from user
oldVersion = raw_input("Enter the old version string: ")

# Read new version number from user
newVersion = raw_input("Enter the new version string: ")
print("ReactPhysics3D will be updated from version " + oldVersion + " to version " + newVersion)

# Replace the RP3D version number in the VERSION file
file = open("VERSION", "w")
file.write(newVersion + "\n")
file.close()
print("Version number has been updated in VERSION file")        

# Update the RP3D version number in the documentation/API/Doxyfile 
findReplaceText("documentation/API/", r'(PROJECT_NUMBER[ \t]+=[ \t]+)"[\d\.]+"', r'\g<1>"' + newVersion + '"', "Doxyfile")
print("Version number has been updated in documentation/API/Doxyfile file")        

# Update the RP3D version number in the src/configuration.h file
findReplaceText("src/", r'(RP3D_VERSION[ \t]+=[ \t]+std::string\()"[\d\.]+"', r'\g<1>"' + newVersion + '"', "configuration.h")
print("Version number has been updated in src/configuration.h file")        

# Update the copyright date in LICENSE file
findReplaceText("./", '(Copyright ' + re.escape("(c)") + r' 2010-)[\d]+', r'\g<1>' + str(date.today().year), "LICENSE")
print("Copyright date has been updated in LICENSE file")        

# Update the copyright date of the license in every source code files
findReplaceText("src/", '(Copyright ' + re.escape("(c)") + r' 2010-)[\d]+', r'\g<1>' + str(date.today().year), "*.h")
findReplaceText("src/", '(Copyright ' + re.escape("(c)") + r' 2010-)[\d]+', r'\g<1>' + str(date.today().year), "*.cpp")
print("Copyright date in license has been updated in all source code files")        


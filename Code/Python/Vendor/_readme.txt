This directory integrates all software that is used by Fluently, but implemented by other Fluently partners

###################################################################################################################
Borzoneg Guglielmo
Generation of cobot paths without collision between cobot+head against table+stander+impeller
D:\Banfi\Github\Fluently\Code\Python\Vendor\Borzoneg

Borzoneg only files (8 MB):
D:\Banfi\Github\Fluently\Code\Python\Vendor\Borzoneg\Borzoneg

Banfi files for Borzoneg 
D:\Banfi\Github\Fluently\Code\Python\Vendor\Borzoneg\Banfi

1)  >cd D:\Banfi\Github\Fluently
	>git clone https://github.com/Borzoneg/knowledge-transfer-fleuntly Code/Python/Vendor/Borzoneg/Borzoneg

2) The cloned package will include its own .git directory. 
To avoid confusion and ensure the vendored code is tracked as part of your own repository (not as a nested repo), 
remove the .git folder inside the vendored package:
>rmdir /s /q "D:\Banfi\Github\Fluently\Code\Python\Vendor\Borzoneg\Borzoneg\.git"


3)	>git add .
	>git commit -m "Added Vendor Borzoneg"
	>git push

5) Use Borzoneg code: from vendor.mypackage import some_module

6) Update the code: do not this with git pull (it won't work since it is not a git repo anymore) but just redo the above git clone step

###################################################################################################################
Orlandini Anneke

Slicer
import slicingAO
import sys


sys.argv=["slicingAO.py", "prova_stl.txt", "1.2", "1"] # nome script slicing, .txt file della parte,layerthickness, Hatching Distance
slices=slicingAO.main()

# Other Input:The user needs to write in command whether mono or bidi strategy

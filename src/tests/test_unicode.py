import sys
sys.path.append("../")

import mutator as m

s = m.rand_unicode(10)
print("|" + s + "|", len(s), type(s))

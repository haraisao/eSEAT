
def handle_add_two_ints(req):
  res = req.a+req.b
  print ("Returning [ %s + %s = %s]" % (req.a, req.b, res))
  return res

def handle_add_two_ints2(req, res):
  res.sum = req.a+req.b
  print ("Returning [ %s + %s = %s]" % (req.a, req.b, res.sum))
  return res

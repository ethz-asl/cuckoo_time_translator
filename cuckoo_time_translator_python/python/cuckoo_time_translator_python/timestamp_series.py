class TimestampSeries(list):

  def __init__(self):
    pass

  def append(self, t):
    assert float == type(t), "type(t)=" + str(type(t))
    list.append(self, t)


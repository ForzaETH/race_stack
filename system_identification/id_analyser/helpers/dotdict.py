# from https://stackoverflow.com/questions/2352181/how-to-use-a-dot-to-access-members-of-dictionary
# DotDict class allows to use dot notation to access dictionary attributes

class DotDict(dict):
  """dot.notation access to dictionary attributes"""
  __getattr__ = dict.get
  __setattr__ = dict.__setitem__
  __delattr__ = dict.__delitem__

  # convert back to normal dict 
  def to_dict(self):
    dict = {}
    for key, value in self.items():
        dict[key] = value
    return dict

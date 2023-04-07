import matlogger2.matlogger as log
import awesome_utils.awesome_pyutils as cal_utils
import matlogger2.matlogger as log_utils

def str2bool(v: str):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")

class OfflineRotDynCal:

  def __init__(self, 
              matpath, 
              window_size, 
              red_ratio, 
              ig_Kt, ig_rot_MoI, ig_Kd0, ig_Kd1):

    self.window_size = window_size
    self.matpath = matpath

    self.opts = log_utils.Options()
    self.opts.load_file_from_path = True
    self.logger = log_utils.MatLogger2(self.matpath, self.opts)

    self.rot_dyn_cal = cal_utils.RotDynCal(self.window_size, 
                                          )

    self.varnames = self.logger.get_varnames()
    


  

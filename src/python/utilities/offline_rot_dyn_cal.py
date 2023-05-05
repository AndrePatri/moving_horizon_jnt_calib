import matlogger2.matlogger as log
import awesome_utils.awesome_pyutils as cal_utils
import matlogger2.matlogger as log_utils
import yaml

def str2bool(v: str):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")

class OfflineRotDynCal:

  def __init__(self, 
              matpath, 
              config_path, 
              verbose = False):

    self.config_path = config_path

    self.matpath = matpath
    
    self.verbose = verbose

    self.load_config()
    self.load_data_from_mat()
    self.init_calibrator()

  def load_config(self):

    with open(self.config_path, "r") as stream:
      try:
          self.mhe_config_file = yaml.safe_load(stream)
      except yaml.YAMLError as exc:
          print(exc)

    self.window_size = self.mhe_config_file["rot_calib_window_size"]
    self.alpha = self.mhe_config_file["alpha"]
    self.q_dot_3sigma = self.mhe_config_file["q_dot_3sigma"]
    self.cal_mask = self.mhe_config_file["cal_mask"]
    self.lambda_reg = self.mhe_config_file["lambda"]
    self.lambda_high = self.mhe_config_file["lambda_high"]
    self.mov_avrg_cutoff_freq = self.mhe_config_file["mov_avrg_cutoff_freq"]
    self.set_ig_to_prev_sol = self.mhe_config_file["set_ig_to_prev_sol"]
    self.jnt_list = self.mhe_config_file["jnt_list"]
    self.red_ratio = self.mhe_config_file["red_ratio"]
    self.K_t_ig = self.mhe_config_file["K_t_ig"]
    self.rot_MoI_ig = self.mhe_config_file["rot_MoI_ig"]
    self.K_d0_ig = self.mhe_config_file["K_d0_ig"]
    self.K_d1_ig = self.mhe_config_file["K_d1_ig"]
    self.K_t_nom = self.mhe_config_file["K_t_nom"]
    self.rot_MoI_nom = self.mhe_config_file["rot_MoI_nom"]
    self.K_d0_nom = self.mhe_config_file["K_d0_nom"]
    self.K_d1_nom = self.mhe_config_file["K_d1_nom"]

  def load_data_from_mat(self):

    self.opts = log_utils.Options()
    self.opts.load_file_from_path = True
    self.logger = log_utils.MatLogger2(self.matpath, self.opts)

    self.varnames = self.logger.get_varnames()

  def init_calibrator(self):

    self.rot_dyn_cal = cal_utils.RotDynCal(self.window_size, 
                                          self.red_ratio, 
                                          self.K_t_ig, 
                                          self.rot_MoI_ig, 
                                          self.K_d0_ig, 
                                          self.K_d1_ig, 
                                          self.lambda_reg,
                                          self.alpha, 
                                          self.q_dot_3sigma, 
                                          self.verbose, 
                                          True)
    
    
    self.rot_dyn_cal.set_lambda_high(self.lambda_high)
    self.rot_dyn_cal.set_solution_mask(self.cal_mask)

  def set_lambda(self, lambda_reg):
     
    self.rot_dyn_cal.set_lambda(lambda_reg)

  def set_lambda_high(self, lambda_high):
     
    self.rot_dyn_cal.set_lambda_high(lambda_high)

  def set_solution_mask(self, cal_mask):
     
    self.rot_dyn_cal.set_solution_mask(cal_mask)

  def fill_window_from_sample(self, index):

    # fill a window of data starting from sample of index i of the 
    # data time series (this method allows to slide through the data back and forth
    # and test the results of the calibration at different times)

    while(not self.rot_dyn_cal.is_window_full()):

      self.rot_dyn_cal.add_sample(q_dot_filt, 
                                  q_ddot_filt, 
                                  iq_meas, 
                                  )

    

  
    


  

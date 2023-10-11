# Copyright (C) 2023  Andrea Patrizi (AndrePatri, andreapatrizi1b6e6@gmail.com)
# 
# This file is part of moving_horizon_jnt_calib and distributed under the General Public License version 2 license.
# 
# moving_horizon_jnt_calib is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# 
# moving_horizon_jnt_calib is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with moving_horizon_jnt_calib.  If not, see <http://www.gnu.org/licenses/>.
# 
import matlogger2.matlogger as log
import awesome_utils.awesome_pyutils as cal_utils
import matlogger2.matlogger as log_utils
import yaml
import numpy as np
import matplotlib.pyplot as plt

def str2bool(v: str):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")

class OfflineRotDynCal:

  def __init__(self, 
              matpath, 
              config_path, 
              verbose = False, 
              use_prev_sol_reg = False, 
              overwrite_window_lentgh = False, 
              window_length = 10000):

    self.overwrite_window_lentgh = overwrite_window_lentgh

    self.config_path = config_path

    self.matpath = matpath
    
    self.verbose = verbose

    self.use_prev_sol_reg = use_prev_sol_reg

    self.load_config()
    self.load_data_from_mat()
    self.init_calibrator()

    self.n_jnts = len(self.red_ratio)

    self.cal_windows = []
    self.kd0_opt = []
    self.kd1_opt =  []
    self.kt_opt = []
    self.rot_moi_opt = []
    self.regr_error = []
    self.sol_millis = []
    self.correlation = []

    self.current_window = {}
  
  class ConvMap:

    def __init__(self, 
                q_dot, 
                q_ddot, 
                tau,
                ref_signal):

      self.q_dot = q_dot
      self.q_ddot = q_ddot
      self.tau = tau

      self.ref_signal = np.transpose(ref_signal)
      
      err = 0
      if (self.q_dot.shape[0] != self.q_ddot.shape[0]):
        err +=1
      if (self.q_ddot.shape[0] != self.tau.shape[0]):
        err +=1
      if (self.tau.shape[0] != self.ref_signal.shape[0]):
        err +=1

      if err != 0:
        raise Exception("OfflineRotDynCal.load_data_from_matThe loaded(): the number of samples in the loaded data do not match!!!") 

      err2 = 0
      if (self.q_dot.shape[1] != self.q_ddot.shape[1]):
        err2 +=1
      if (self.q_ddot.shape[1] != self.tau.shape[1]):
        err2 +=1
      if (self.tau.shape[1] != self.ref_signal.shape[1]):
        err2 +=1

      if err2 != 0:
        raise Exception("OfflineRotDynCal.load_data_from_matThe loaded(): the number of joints in the loaded data do not match!!!") 

      self.n_jnts = self.q_dot.shape[0]
      self.n_samples = self.q_dot.shape[1]

      self.conv_map = {}
      self.conv_map["q_dot"] = np.zeros((self.n_jnts))
      self.conv_map["q_ddot"] = np.zeros((self.n_jnts))
      self.conv_map["tau"] = np.zeros((self.n_jnts))

      self.compute_convolution()

    def compute_convolution(self):
      
      self.q_dot_nom = self.normalize(self.q_dot)
      self.q_ddot_norm = self.normalize(self.q_ddot)
      self.tau_norm = self.normalize(self.tau)
      self.ref_signal_norm = self.normalize(self.ref_signal)

      for i in range(self.n_jnts):

        self.conv_map["q_dot"][i] = (np.dot(self.q_dot_nom[i, :], self.ref_signal_norm[i, :])) / self.n_samples
        self.conv_map["q_ddot"][i] = (np.dot(self.q_ddot_norm[i, :] , self.ref_signal_norm[i, :])) / self.n_samples
        self.conv_map["tau"][i] = (np.dot(self.tau_norm[i, :], self.ref_signal_norm[i, :])) / self.n_samples
    
    def normalize(self, signal):

        scale_max = np.amax(signal, axis = 1)
        scale_min = np.amin(signal, axis = 1)

        signal_norm = np.zeros((signal.shape[0], signal.shape[1]))

        for i in range(0, self.n_jnts):
          
          scale = 1

          if (abs(scale_max[i]) > abs(scale_min[i])):

            scale = abs(scale_max[i])
          
          else:

            scale = abs(scale_min[i])

          signal_norm[i, :] = signal[i, :] / scale

        return signal_norm

  def load_config(self):

    with open(self.config_path, "r") as stream:
      try:
          self.mhe_config_file = yaml.safe_load(stream)
      except yaml.YAMLError as exc:
          print(exc)

    if not self.overwrite_window_lentgh:    
      self.window_size = self.mhe_config_file["rot_calib_window_size"]
    else:
      self.window_size = window_length

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

    self.q_dot = self.logger.readvar("q_p_dot_meas")
    self.q_ddot = self.logger.readvar("q_p_ddot_meas")
    self.tau_meas = self.logger.readvar("tau_meas")
    self.iq_meas = self.logger.readvar("iq_meas")

    self.loop_time = self.logger.readvar("loop_time")

    err = 0
    if (self.q_dot.shape[1] != self.q_ddot.shape[1]):
      err +=1
    if (self.q_ddot.shape[1] != self.tau_meas.shape[1]):
      err +=1
    if (self.tau_meas.shape[1] != self.iq_meas.shape[1]):
      err +=1

    if err != 0:
      raise Exception("OfflineRotDynCal.load_data_from_matThe loaded(): the number of samples in the loaded data do not match!!!") 
    
  def init_calibrator(self):

    self.calibrator = cal_utils.RotDynCal(self.window_size, 
                                          np.reciprocal(np.array(self.red_ratio).astype(float)), 
                                          self.K_t_nom, 
                                          self.rot_MoI_nom, 
                                          self.K_d0_nom, 
                                          self.K_d1_nom, 
                                          self.lambda_reg,
                                          self.alpha, 
                                          self.q_dot_3sigma, 
                                          self.verbose, 
                                          True)
    
    self.calibrator.set_lambda_high(self.lambda_high)
    self.calibrator.set_solution_mask(self.cal_mask)

  def set_lambda(self, lambda_reg):
     
    self.calibrator.set_lambda(lambda_reg)

  def set_lambda_high(self, lambda_high):
     
    self.calibrator.set_lambda_high(lambda_high)

  def set_solution_mask(self, cal_mask):
     
    self.calibrator.set_solution_mask(cal_mask)

  def fill_window_from_sample(self, index):

    # fill a window of data starting from sample of index i of the 
    # data time series (this method allows to slide through the data back and forth
    # and test the results of the calibration at different times)

    self.calibrator.reset_window() # we tell the calibrator that
    # we want to fill the window again (in case it was already filled with 
    # other data)
    self.current_window_index = index
    i = self.current_window_index
    while(not self.calibrator.is_window_full()):
      
      # we stop adding samples when the window is full
      self.calibrator.add_sample(self.q_dot[:, i], 
                                  self.q_ddot[:, i],
                                  self.iq_meas[:, i],
                                  self.tau_meas[:, i])
      
      i += 1
      
    self.current_window["q_dot"] = \
      self.q_dot[:, self.current_window_index: self.current_window_index + self.window_size]
    self.current_window["q_ddot"] = \
      self.q_ddot[:, self.current_window_index: self.current_window_index + self.window_size]
    self.current_window["tau_meas"] = \
      self.tau_meas[:, self.current_window_index: self.current_window_index + self.window_size]
    self.current_window["iq_meas"] = \
      self.iq_meas[:, self.current_window_index: self.current_window_index + self.window_size]

  def run_calibration(self):

    if(self.calibrator.is_window_full()):
      
      self.calibrator.solve()

      self.cal_windows.append(self.current_window)

      self.calibrator.reset_window() # we force a call to fill_window_from_sample
      # before the solve method can be called again

      if (self.use_prev_sol_reg):
        
        self.calibrator.set_ig_Kd0(self.calibrator.get_opt_Kd0())
        self.calibrator.set_ig_Kd1(self.calibrator.get_opt_Kd1())
        self.calibrator.set_ig_Kt(self.calibrator.get_opt_Kt())
        self.calibrator.set_ig_MoI(self.calibrator.get_opt_rot_MoI())
    
    else:

      print("run_calibration(): calibration will not be run. You have to finish filling the data window first!")

  def add_solution_data(self):

    current_regr_error = self.calibrator.get_regr_error()

    self.regr_error.append(current_regr_error)
    
    self.correlation.append(OfflineRotDynCal.ConvMap(self.q_dot[:, self.current_window_index: self.current_window_index + self.window_size], 
                                    self.q_ddot[:, self.current_window_index: self.current_window_index + self.window_size], 
                                    self.tau_meas[:, self.current_window_index: self.current_window_index + self.window_size], 
                                    current_regr_error))

    print(self.correlation[0].conv_map["q_dot"])
    print(self.correlation[0].conv_map["q_ddot"])
    print(self.correlation[0].conv_map["tau"])

    exit()

    self.sol_millis.append(self.calibrator.get_sol_millis())

    self.kd0_opt.append(self.calibrator.get_opt_Kd0())
    self.kd1_opt.append(self.calibrator.get_opt_Kd1())
    self.kt_opt.append(self.calibrator.get_opt_Kt())
    self.rot_moi_opt.append(self.calibrator.get_opt_rot_MoI())
    
  def make_plots(self):

    fontsize = 14
    f, ax = plt.subplots(1)

    ax.plot(self.loop_time[0, self.current_window_index:self.current_window_index + self.window_size], self.regr_error[0], drawstyle='steps-post')

    ax.set_xlabel(r'[$\mathrm{s}$]',  fontsize=fontsize)
    ax.set_ylabel(r'[$\mathrm{N}$]', fontsize=fontsize)
    ax.set_title(r"$\mathrm{MHE\,regression\,error}$", fontsize=fontsize)
    ax.grid()
    legend = ax.legend([r"$\mathrm{regr}_{\mathrm{err}}$"], fontsize=fontsize)
    legend.set_draggable(state = True)

    plt.show()

  
    


  

import matlab.engine
eng = matlab.engine.start_matlab()
# eng.run('generate_model_code.m', nargout=0)
eng.run('Rebuild_Model.m', nargout=0)

eng.quit()
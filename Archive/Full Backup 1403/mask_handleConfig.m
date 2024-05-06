classdef mask_handleConfig

    methods (Static)
        function extractData(callbackContext)
            blkHandle = callbackContext.BlockHandle;
            maskObj = Simulink.Mask.get(blkHandle);
            parameterObj = callbackContext.ParameterObject;
        
            % extract data from configuration object
            config = maskWorkspace.get('config');
            I = mat2str(config.I);

            % set variables in mask
            maskObj.getParameter('I').Value = I
        end


    end
end
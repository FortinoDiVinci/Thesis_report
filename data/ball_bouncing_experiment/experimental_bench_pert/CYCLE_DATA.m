classdef CYCLE_DATA < handle
    
    properties 
        idx;        % indexes in the global data set
        time;       % t(idx)
        position;   % p(idx)
        velocity;   % v(idx)
        isPerturbed;
        pert_val;
        pert_dir; 
        pert_t;
        pert_idx;
    end
    
    methods
        %
        function self = CYCLE_DATA(time, position, velocity, indexes)
            switch (nargin)
                case 0
                    % default constructor
                    self.time = [];
                    self.position = [];
                    self.velocity = [];
                    self.idx = [];
                case 1
                    error('Not enough inputs provided')
                case 2
                    self.time = time;
                    self.position = position;
                    self.velocity = [];
                    self.idx = [];
                case 3
                    self.time = time;
                    self.position = position;
                    self.velocity = velocity;
                    self.idx = [];
                case 4
                    self.time = time;
                    self.position = position;
                    self.velocity = velocity;
                    self.idx = indexes;
                otherwise
                    warning('Too many inputs provided')
                    self.time = time;
                    self.position = position;
                    self.velocity = velocity;
                    self.idx = indexes;
            end
            self.isPerturbed = 0;
            self.pert_val = 0;
            self.pert_dir = 0;
        end
        %
        function self = setPerturbation(self, perturbation_value, perturbation_time)
            if perturbation_value ~= 0
                self.isPerturbed = 1;
                self.pert_val = perturbation_value;
                self.pert_dir = sign(perturbation_value);
                self.pert_t = perturbation_time;
                self.pert_idx = find(self.pert_t >= self.time, 1, 'first');
            end
        end
        %
        % (c) Doug Schwarz, dmschwarz@ieee.org
        % Make a copy of a handle object.
        function new = copy(self)
            % Instantiate new object of the same class.
            new = feval(class(self));
            % Copy all non-hidden properties.
            p = properties(self);
            for i = 1:length(p)
                new.(p{i}) = self.(p{i});
            end
        end
    end
    
end
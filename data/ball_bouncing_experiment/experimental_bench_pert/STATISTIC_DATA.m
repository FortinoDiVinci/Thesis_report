classdef STATISTIC_DATA < handle
    
    properties   
        data;
        name;
        unit;
        %
        idx; % list of the indexes for the statistical computations
        mean; % list of means (according to the indexes)
        median; % list of medians (according to the indexes)
        std_dev; % list of standard deviations (according to the indexes)
    end
    
    methods
        %
        function self = STATISTIC_DATA(data, name, unit)
            switch (nargin)
                case 0
                    %
                    self.data = [];
                case 1
                    self.data = data;
                case 2
                    self.data = data;
                    self.name = name;
                case 3
                    self.data = data;
                    self.name = name;
                    self.unit = unit;
                otherwise
                    warning('Too many inputs provided')
                    self.data = data;
                    self.name = name;
                    self.unit = unit;
            end
            self.idx = {};
            self.mean = [];
            self.median = [];
            self.std_dev = [];
        end
        %
        function test = isDataEmpty(self)
                test = any(~size(self.data)); % if any dimension is 0
        end
        %
        function self = set_data(self, data)
          	self.data = data;
        end
        %
        function self = set_name(self, name)
          	self.name = name;
        end
        %
        function self = set_unit(self, unit)
          	self.unit = unit;
        end
        %
        function self = compute_new_statistics(self, idx)
            if self.isDataEmpty()
                warning('The data set provided is empty')
            else
                if nargin > 1
                    self.idx{end+1} = idx;
                    self.mean = [self.mean, nanmean(self.data(idx))];
                    self.median = [self.median, nanmedian(self.data(idx))];
                    self.std_dev = [self.std_dev, nanstd(self.data(idx))];
                else
                    self.idx = [self.idx, (1:length(self.data))'];
                    self.mean = [self.mean, nanmean(self.data)];
                    self.median = [self.median, nanmedian(self.data)];
                    self.std_dev = [self.std_dev, nanstd(self.data)];
                end
            end
        end
        %
        function self = clear_statistics(self, idx)
            if nargin > 1
                self.idx{idx} = [];
                self.mean(idx) = [];
                self.median(idx) = [];
                self.std_dev(idx) = [];
            else
                self.idx = {};
                self.mean = [];
                self.median = [];
                self.std_dev = [];
            end
        end
    end
    
end
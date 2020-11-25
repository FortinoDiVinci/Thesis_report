clear all
close all

T = readtable('data_2020_Nov_17_concatenated.csv');

Time = T.Time;
subData = [];
subData_size = 1500;
j = 1;

for it = 1:floor(length(T.Position)/subData_size)
    
    idx = (1:1500)+(it-1)*subData_size;
    
    % Avoid mixing two different trials in a single chunck
    if(any(diff(T.TrialNb(idx))))
        continue
    end
    
    % Avoid using cases where the ball was not moving yet
    if(all(~diff(T.Ball(idx))))
        continue
    end
    
    subData(j).position = T.Position(idx);
    subData(j).force = T.Force(idx);
    subData(j).ball = T.Ball(idx);
    if T.Hand(idx(1))
        subData(j).hand = "right";
    else
        subData(j).hand = "left";
    end
    subData(j).isBallImpact = T.IsBallImpact(idx(1));
    
    j = j + 1;
    
end

save('youBotRhythmicTaskSubTrajectories.mat','subData')

rand_idx = randperm(length(subData));

figure
for it = 1:9
    idx = rand_idx(it);
    subplot(3,3,it)
    plot(subData(idx).position)
    hold on
    plot(subData(idx).ball)
    yyaxis right
    plot(subData(idx).force)
    if contains(subData(idx).hand, "right")
        hand = "right hand.";
    else
        hand = "left hand.";
    end
    if subData(idx).isBallImpact
        b_i = ", with";
    else
        b_i = ", without";
    end
    title("Trial n" + num2str(idx) + b_i + " ball haptic feedback, using the " + hand)
end
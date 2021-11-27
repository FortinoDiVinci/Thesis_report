age=[27,27,26,25,27,27,24,26,26,40,24,34,50,30,28,26,22,49,30,27,27,25,26,19,27,29,43,27,56,24,31];
size=[175,183,176,178,174,183,192,165,180,163,157,175,176,188,185,177,175,166,174,162,174,179,172,180,162,200,182,172,181,185,170];
weight=[66,73,75,67,70,70,85,59,95,53,48,65,72,68,75,90,60,80,68,61,58,66,70,67,65,95,105,66,78,80,60];
BMI = weight./(size/100).^2;


figure
subplot(1,3,1)
histogram(age)
subplot(1,3,2)
histogram(weight,'BinWidth',5)
subplot(1,3,3)
histogram(size,'BinWidth',5)

figure
histogram(BMI)

mean(age)
median(age)
prctile(age,[25,75])

mean(weight)
median(weight)
std(weight)
prctile(weight,[25,75])

mean(size)
median(size)
std(size)
prctile(size,[25,75])

mean(BMI)
median(BMI)
std(BMI)
prctile(BMI,[25,75])
function [new_mean,new_var] = updatef4 (mean1,var1,mean2,var2,mean3,var3,mean4,var4)
        new_mean = (mean1/var1 + mean2/var2 + mean3/var3 + mean4/var4) / (1/var1 + 1/var2 + 1/var3 + 1/var4);
        new_var = 1/(1/var1+1/var2+1/var3+1/var4);
       
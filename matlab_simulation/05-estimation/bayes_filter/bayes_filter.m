% A discrete bayes filter algorithm
% For very simple estimation problems only

function bfout = bayes_filter(prob_motion, prob_meas, pred_pri)

% Prediction update (integration as a finite sum)
pred_upd = prob_motion(:,:) * pred_pri(:,:);

% Measurement update
meas_upd = prob_meas.*pred_upd;
% Normalize
meas_upd = meas_upd/norm(meas_upd);

bfout = [pred_upd meas_upd];
end
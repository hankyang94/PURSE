function out = is_pose_in_purse(R,t,A,B)

p = [R(:);t];

out = true;
for i = 1:size(A,1)
    Ai = squeeze(A(i,:,:));
    if p'*Ai*p > 1e-3
        out = false;
    end
end
for i = 1:size(B,1)
    Bi = squeeze(B(i,:));
    Bi = Bi(:);
    if p'*Bi <= 0
        out = false;
    end
end
end
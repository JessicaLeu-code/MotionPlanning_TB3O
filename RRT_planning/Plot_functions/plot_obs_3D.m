function [plot_marker] = plot_obs_3D(obs,view_area)
num_obs = size(obs,2); 
plot_marker=[];
for j=1:num_obs
    if ~isempty(obs{j})
        switch obs{j}.shape
            case 'box'
                plotcube(obs{j}.region'*2,obs{j}.c'-obs{j}.region',.8,[1 0 0]);
                hold on
            case 'cylinder'            
                r = obs{j}.D/2;
                [X,Y,Z] = cylinder(r);
                plot_marker(j) = surf(X+obs{j}.l(1,1),Y+obs{j}.l(2,1),Z*(obs{j}.l(3,2)-obs{j}.l(3,1))+obs{j}.l(3,1));
                hold on
                [X,Y,Z] = sphere;
                surf(X*r+obs{j}.l(1,1),Y*r+obs{j}.l(2,1),Z*r+obs{j}.l(3,1));
                surf(X*r+obs{j}.l(1,1),Y*r+obs{j}.l(2,1),Z*r+obs{j}.l(3,2));
                
            case 'plate'            
                ob = Polyhedron('V',obs{j}.poly');
                plot_marker(j) = ob.plot('color','g');
                hold on
        end
    end
    axis(view_area)
    axis equal
    view(2)
end

end
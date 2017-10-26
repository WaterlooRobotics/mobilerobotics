function listObject = linked_List(values)
  if nargin == 0
    data = [];
    listObject = struct('display',@display_list,...
                      'SetTop',@setTop_element,...
                      'GetTop',@getTop_element,...
                      'SetBottom',@setBottom_element,...
                      'GetBottom',@getBottom_element,...
                      'Add',@add_element,...
                      'Cnt',@count_element,...
                      'ShortDistToFinish', @Shortest_Index_To_Finish_element,...
                      'Remove',@remove_element,...
                      'Get',@get_element,...
                      'delete',@delete_element);

      return
  end
  
  data = reshape(values,1,[]);
  listObject = struct('display',@display_list,...
                      'SetTop',@setTop_element,...
                      'GetTop',@getTop_element,...
                      'SetBottom',@setBottom_element,...
                      'GetBottom',@getBottom_element,...
                      'Add',@add_element,...
                      'Cnt',@count_element,...
                      'ShortDistToFinish', @Shortest_Index_To_Finish_element,...
                      'Remove',@remove_element,...
                      'Get',@get_element,...
                      'delete',@delete_element);

  function display_list(index)
    %# Displays the data in the list
    if(index <= numel(data))
       disp(data(index));
    end
  end

  function values= get_element(index)

    if(index <= numel(data))
       values = data(index);
    end
  end

  function cnt=count_element()
    %return the number of elements
    cnt= numel(data);
  end

  function index=Shortest_Index_To_Finish_element()
    %return the number of elements
    cnt= numel(data);
    if cnt <1 then
        index = 0;
        return;
    end
    minDist=1e10;
    for i=1:cnt
        if data(i).distToFinish < minDist
           index = i 
           minDist = data(i).distToFinish;
        end    
    end
  end


  function setTop_element(values,index)
    %# Adds a set of data values after an index in the list, or at the end
    %#   of the list if the index is larger than the number of list elements
    %index = min(index,numel(data));
    data = [reshape(values,1,[]) data(1:end)];
  end

  function value= getTop_element()
    value = data(1);
    if(numel(data) >1)
        data = [data(2:end)];
    end
  end
  
  function setBottom_element(values)
    data = [data(1:end) reshape(values,1,[])];
  end

  function getBottom_element(values)
    data = [reshape(values,1,[]) data(1:end)];
    listObject = data(end);
    if(numel(data) >1)
        data = [data(1:end-1)];
    end
  end
  
  function add_element(values,index)
    %# Adds a set of data values after an index in the list, or at the end
    %#   of the list if the index is larger than the number of list elements
    index = min(index,numel(data));
    data = [data(1:index) reshape(values,1,[]) data(index+1:end)];
  end

  function remove_element(values)  
      for i=1:numel(data)
         if(data(i).index == values.index)
            delete_element(i);
            return;
         end
      end
  end    
      
  function delete_element(index)
    %# Deletes an element at an index in the list
    data(index) = [];
  end
end

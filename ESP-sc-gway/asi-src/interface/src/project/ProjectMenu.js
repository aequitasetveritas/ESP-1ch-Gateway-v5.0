import React, { Component } from 'react';
import { Link, withRouter } from 'react-router-dom';

import { PROJECT_PATH } from '../constants/Env';

import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import SettingsRemoteIcon from '@material-ui/icons/SettingsRemote';
import FastForwardIcon from '@material-ui/icons/FastForward';
import SimCardIcon from '@material-ui/icons/SimCard';

class ProjectMenu extends Component {

  render() {
    const path = this.props.match.url;
    return (
      <List>
        <ListItem to={`/${PROJECT_PATH}/demo/`} selected={path.startsWith(`/${PROJECT_PATH}/demo/`)} button component={Link}>
          <ListItemIcon>
            <SettingsRemoteIcon />
          </ListItemIcon>
          <ListItemText primary="Demo Project" />
        </ListItem>

        <ListItem to={`/${PROJECT_PATH}/lora/`} selected={path.startsWith(`/${PROJECT_PATH}/lora/`)} button component={Link}>
          <ListItemIcon>
            <SettingsRemoteIcon />
          </ListItemIcon>
          <ListItemText primary="Lora Gateway" />
        </ListItem>

        <ListItem to={`/${PROJECT_PATH}/pf/`} selected={path.startsWith(`/${PROJECT_PATH}/pf/`)} button component={Link}>
          <ListItemIcon>
            <FastForwardIcon />
          </ListItemIcon>
          <ListItemText primary="Packet Forwarder" />
        </ListItem>

        <ListItem to={`/${PROJECT_PATH}/gprs/status`} selected={path.startsWith(`/${PROJECT_PATH}/gprs/status`)} button component={Link}>
          <ListItemIcon>
            <SimCardIcon />
          </ListItemIcon>
          <ListItemText primary="GPRS" />
        </ListItem>

      </List>
    )
  }

}

export default withRouter(ProjectMenu);
